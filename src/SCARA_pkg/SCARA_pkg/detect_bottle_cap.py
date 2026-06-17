#!/usr/bin/env python3

import os

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from ultralytics import YOLO

# ── Topics ─────────────────────────────────────────────────────────────────────
TOPIC_DETECTIONS = "/bottle_cap/detections"   # PointStamped in camera optical frame (m)
TOPIC_IMAGE      = "/bottle_cap/image"        # annotated colour image

# Frame the published point is expressed in. The launch file must publish a
# static TF from base_link → this frame so the C++ node can transform it.
CAMERA_OPTICAL_FRAME = "camera_color_optical_frame"

# ── Stability filter ───────────────────────────────────────────────────────────
# Only publish a detection once the same cap has stayed within STABILITY_TOL_PX
# pixels of its first-seen position for STABILITY_DURATION_S seconds. Prevents
# the robot from chasing a cap that's still being moved.
STABILITY_TOL_PX     = 15
STABILITY_DURATION_S = 5.0

# ── Digital zoom ───────────────────────────────────────────────────────────────
# Centre-crop the colour frame and upscale before YOLO (and before display).
# Improves detection of small/distant caps without moving the camera. The
# published cx/cy stay in the ORIGINAL frame's coordinate system. Set to 1.0
# to disable.
ZOOM_FACTOR = 2.0

# ── Model path ─────────────────────────────────────────────────────────────────
# Defaults to the canonical workspace layout (~/scara_bot_ws/src/best_v2.pt).
# Override per-run with:
#   ros2 run SCARA_pkg detect_bottle_cap --ros-args -p model_path:=/abs/path/best_v2.pt
DEFAULT_MODEL_PATH = "~/scara_bot_ws/src/best_v2.pt"


class BottleCapDetectorNode(Node):
    """
    Streams RealSense colour + depth frames, runs YOLO bottle_cap detection,
    and publishes the centre pixel (cx, cy) of the highest-confidence stable
    cap as Int32MultiArray on /bottle_cap/detections.
    """

    def __init__(self):
        super().__init__("bottle_cap_detector")

        # ── Parameters
        self.declare_parameter("show_preview", True)
        self._show_preview = self.get_parameter("show_preview").get_parameter_value().bool_value

        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        model_path = os.path.expanduser(
            self.get_parameter("model_path").get_parameter_value().string_value
        )

        # ── Publishers
        self._det_pub = self.create_publisher(PointStamped, TOPIC_DETECTIONS, 10)
        self._img_pub = self.create_publisher(Image, TOPIC_IMAGE, 10)
        self._bridge  = CvBridge()

        # ── YOLO model
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO loaded from {model_path}  |  task: {self.model.task}")

        # ── RealSense pipeline
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self._profile = self._pipeline.start(cfg)
        self._align   = rs.align(rs.stream.color)

        # ── Cache colour-stream intrinsics (used for deprojection)
        self._intr = (
            self._profile
            .get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        self.get_logger().info(
            f"Color intrinsics  fx={self._intr.fx:.2f}  fy={self._intr.fy:.2f}  "
            f"ppx={self._intr.ppx:.2f}  ppy={self._intr.ppy:.2f}"
        )

        # ── Stability tracking state
        self._stable_anchor_px = None
        self._stable_since_s   = None
        self._stable_published = False

        # ── 30 Hz processing loop
        self.create_timer(1.0 / 30.0, self._callback)
        self.get_logger().info(
            f"BottleCapDetectorNode ready  "
            f"|  stability: {STABILITY_TOL_PX}px tol, {STABILITY_DURATION_S:.1f}s hold"
        )

    # ── Main loop ──────────────────────────────────────────────────────────────

    def _callback(self):
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError:
            return

        frames      = self._align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        color_img = np.asanyarray(color_frame.get_data())

        # ── Digital zoom
        H, W = color_img.shape[:2]
        if ZOOM_FACTOR > 1.0:
            crop_w = int(W / ZOOM_FACTOR)
            crop_h = int(H / ZOOM_FACTOR)
            x_off  = (W - crop_w) // 2
            y_off  = (H - crop_h) // 2
            cropped  = color_img[y_off:y_off + crop_h, x_off:x_off + crop_w]
            view_img = cv2.resize(cropped, (W, H), interpolation=cv2.INTER_LINEAR)
        else:
            x_off, y_off = 0, 0
            view_img = color_img

        # ── YOLO inference (on the zoomed view)
        result = self.model(view_img, conf=0.25, iou=0.45, verbose=False)[0]

        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                zx1, zy1, zx2, zy2 = map(int, box.xyxy[0].tolist())
                conf   = float(box.conf[0])
                cls_id = int(box.cls[0])
                name   = result.names[cls_id]

                # Map zoomed pixels back to ORIGINAL colour frame.
                x1 = int(x_off + zx1 / ZOOM_FACTOR)
                y1 = int(y_off + zy1 / ZOOM_FACTOR)
                x2 = int(x_off + zx2 / ZOOM_FACTOR)
                y2 = int(y_off + zy2 / ZOOM_FACTOR)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # Annotate the zoomed view (depth shown for operator reference only).
                depth_m = depth_frame.get_distance(cx, cy)
                zcx = (zx1 + zx2) // 2
                zcy = (zy1 + zy2) // 2
                self._annotate(view_img, zx1, zy1, zx2, zy2,
                               zcx, zcy, name, conf, depth_m)

                detections.append({
                    "confidence": conf,
                    "center_px":  (cx, cy),
                })

        # Stability gate — publish only after STABILITY_DURATION_S of holding.
        self._maybe_publish_stable(detections, depth_frame, view_img, x_off, y_off)

        # Annotated image stream
        self._img_pub.publish(self._bridge.cv2_to_imgmsg(view_img, encoding="bgr8"))

        if self._show_preview:
            cv2.imshow("Bottle Cap Detector", view_img)
            cv2.waitKey(1)

    # ── Stability gating ───────────────────────────────────────────────────────

    def _maybe_publish_stable(self, detections: list, depth_frame,
                              overlay_img: np.ndarray,
                              x_off: int = 0, y_off: int = 0):
        if not detections:
            if self._stable_anchor_px is not None:
                self.get_logger().info("Stability reset: cap lost.")
            self._stable_anchor_px = None
            self._stable_since_s   = None
            self._stable_published = False
            return

        best   = max(detections, key=lambda d: d["confidence"])
        cx, cy = best["center_px"]
        now    = self.get_clock().now().nanoseconds * 1e-9

        if self._stable_anchor_px is None:
            self._stable_anchor_px = (cx, cy)
            self._stable_since_s   = now
            self._stable_published = False
        else:
            ax, ay = self._stable_anchor_px
            if (cx - ax) ** 2 + (cy - ay) ** 2 > STABILITY_TOL_PX ** 2:
                self.get_logger().info(
                    f"Stability reset: moved from ({ax},{ay}) to ({cx},{cy})."
                )
                self._stable_anchor_px = (cx, cy)
                self._stable_since_s   = now
                self._stable_published = False

        elapsed   = now - self._stable_since_s
        remaining = STABILITY_DURATION_S - elapsed

        # Lock-circle overlay (in zoomed-view coords).
        ax, ay  = self._stable_anchor_px
        zax     = int((ax - x_off) * ZOOM_FACTOR)
        zay     = int((ay - y_off) * ZOOM_FACTOR)
        zradius = int(STABILITY_TOL_PX * ZOOM_FACTOR)
        if remaining > 0:
            label, colour = f"locking… {remaining:0.1f}s", (0, 165, 255)
        else:
            label, colour = "LOCKED", (0, 255, 0)
        cv2.circle(overlay_img, (zax, zay), zradius, colour, 1)
        cv2.putText(overlay_img, label, (zax + 10, zay - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

        if elapsed < STABILITY_DURATION_S:
            return

        # Stable — emit the locked pixel as a ray in the camera optical
        # frame. We publish the point at unit Z along the ray:
        #   X = (px - ppx) / fx,  Y = -(py - ppy) / fy,  Z = 1.0
        # Y is negated to match the camera's physical mount orientation
        # (image-down maps to +base_link Y instead of -Y). The downstream
        # C++ node intersects this ray with the known workspace plane in
        # base_link, so we don't depend on depth at all (unreliable on
        # matte black surfaces with the D435i IR).
        X = (float(cx) - self._intr.ppx) / self._intr.fx
        Y = -(float(cy) - self._intr.ppy) / self._intr.fy
        Z = 1.0

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = CAMERA_OPTICAL_FRAME
        msg.point.x = float(X)
        msg.point.y = float(Y)
        msg.point.z = float(Z)
        self._det_pub.publish(msg)

        if not self._stable_published:
            self.get_logger().info(
                f"Stable cap at px=({cx},{cy}) → ray in "
                f"{CAMERA_OPTICAL_FRAME}=({X:.4f}, {Y:.4f}, 1.0) (unit-Z)"
            )
            self._stable_published = True

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _annotate(img, x1, y1, x2, y2, cx, cy, name, conf, depth_m):
        colour = (0, 220, 160)
        cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
        cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)
        cv2.putText(
            img, f"{name}  {conf:.2f}  d={depth_m:.3f}m",
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2,
        )
        cv2.putText(
            img, f"px=({cx},{cy})",
            (x1, y2 + 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1,
        )

    def destroy_node(self):
        try:
            self._pipeline.stop()
        except Exception:
            pass
        if self._show_preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BottleCapDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
