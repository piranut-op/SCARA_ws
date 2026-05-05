#!/usr/bin/env python3

import json

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

# ── Topics ─────────────────────────────────────────────────────────────────────
TOPIC_DETECTIONS = "/bottle_cap/detections"       # raw pixel + depth detections
TOPIC_WORKSPACE  = "/bottle_cap/workspace_position"  # localized workspace coords
TOPIC_IMAGE      = "/bottle_cap/image"            # annotated colour image

# ── Physical setup ─────────────────────────────────────────────────────────────
WORKSPACE_W_CM  = 16.0   # workspace width  (x-axis) in cm
WORKSPACE_H_CM  = 18.0   # workspace height (y-axis) in cm
CAMERA_H_M      = 0.50   # nominal camera height above workspace plane (metres)

# ── Model path ─────────────────────────────────────────────────────────────────
# Edit this to point at your bottle_cap best.pt
MODEL_PATH = "/home/piranut/scara_bot_ws/src/best_v2.pt"


# NODE 1 — Bottle Cap Detector

class BottleCapDetectorNode(Node):
    """
    Streams RealSense colour + depth frames, runs YOLO bottle_cap detection,
    and publishes raw detections (pixel position + real depth) as JSON.
    """

    def __init__(self):
        super().__init__("bottle_cap_detector")

        # ── Parameters (override from CLI / launch file)
        self.declare_parameter("show_preview", True)
        self._show_preview = self.get_parameter("show_preview").get_parameter_value().bool_value

        # ── Publishers
        self._det_pub = self.create_publisher(String, TOPIC_DETECTIONS, 10)
        self._img_pub = self.create_publisher(Image,  TOPIC_IMAGE,      10)
        self._bridge  = CvBridge()

        # ── YOLO model (your bottle_cap weights)
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"YOLO loaded from {MODEL_PATH}  |  task: {self.model.task}")

        # ── RealSense pipeline
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self._profile = self._pipeline.start(cfg)
        self._align   = rs.align(rs.stream.color)   # align depth → colour frame

        # ── Cache camera intrinsics (constant after start)
        self._intr     = (
            self._profile
            .get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        self._intr_dict = {
            "fx":  self._intr.fx,
            "fy":  self._intr.fy,
            "ppx": self._intr.ppx,
            "ppy": self._intr.ppy,
        }
        self.get_logger().info(
            f"Intrinsics  →  fx={self._intr.fx:.1f}  fy={self._intr.fy:.1f}  "
            f"ppx={self._intr.ppx:.1f}  ppy={self._intr.ppy:.1f}"
        )

        # ── 30 Hz processing loop
        self.create_timer(1.0 / 30.0, self._callback)
        self.get_logger().info("BottleCapDetectorNode ready.")

    # ── Main loop ──────────────────────────────────────────────────────────────

    def _callback(self):
        # Grab aligned frames (non-blocking via timeout)
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError:
            return  # timed out — skip cycle

        frames      = self._align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        color_img = np.asanyarray(color_frame.get_data())

        # ── YOLO inference
        result = self.model(color_img, conf=0.25, iou=0.45, verbose=False)[0]

        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf   = float(box.conf[0])
                cls_id = int(box.cls[0])
                name   = result.names[cls_id]

                # Pixel centre of bounding box
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # Depth at centre pixel (fall back to nominal height if invalid)
                depth_m = depth_frame.get_distance(cx, cy)
                if depth_m <= 0.0:
                    depth_m = CAMERA_H_M

                # Annotate frame
                self._annotate(color_img, x1, y1, x2, y2,
                               cx, cy, name, conf, depth_m)

                detections.append({
                    "class":      name,
                    "confidence": round(conf, 4),
                    "box":        [x1, y1, x2, y2],
                    "center_px":  [cx, cy],
                    "depth_m":    round(depth_m, 5),
                    "intrinsics": self._intr_dict,
                })

        # Publish detections
        if detections:
            msg      = String()
            msg.data = json.dumps(detections)
            self._det_pub.publish(msg)

        # Publish annotated image
        self._img_pub.publish(
            self._bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
        )

        # Optional local preview (skip in headless environments)
        if self._show_preview:
            cv2.imshow("Bottle Cap Detector", color_img)
            cv2.waitKey(1)

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _annotate(img, x1, y1, x2, y2, cx, cy, name, conf, depth_m):
        """Draw bounding box, centre dot, and label onto the colour image."""
        colour = (0, 220, 160)   # teal-green for bottle caps
        cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
        cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)
        cv2.putText(
            img,
            f"{name}  {conf:.2f}  d={depth_m:.3f}m",
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2,
        )
        # Mark pixel centre coordinates on image
        cv2.putText(
            img,
            f"px=({cx},{cy})",
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

# NODE 2 — Workspace Localizer

class WorkspaceLocalizerNode(Node):

    def __init__(self):
        super().__init__("workspace_localizer")

        self._sub = self.create_subscription(
            String, TOPIC_DETECTIONS, self._on_detection, 10
        )
        self._pub = self.create_publisher(String, TOPIC_WORKSPACE, 10)
        self.get_logger().info(
            f"WorkspaceLocalizerNode ready  "
            f"|  workspace {WORKSPACE_W_CM}x{WORKSPACE_H_CM} cm"
        )

    # ── Subscriber callback ────────────────────────────────────────────────────

    def _on_detection(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received malformed detection JSON — skipping.")
            return

        results = [self._localize(det) for det in detections]

        out      = String()
        out.data = json.dumps(results, indent=None)
        self._pub.publish(out)

        for r in results:
            self._log_result(r)

    # ── Coordinate conversion pipeline ────────────────────────────────────────

    def _localize(self, det: dict) -> dict:
        """Full pixel → camera-3D → workspace → robot conversion for one detection."""
        cx, cy  = det["center_px"]
        depth_m = det["depth_m"]
        intr    = det["intrinsics"]

        # Step 1 — Deproject pixel to camera-frame 3-D point
        X, Y, Z = self._pixel_to_cam3d(cx, cy, depth_m,
                                        intr["fx"], intr["fy"],
                                        intr["ppx"], intr["ppy"])

        # Step 2 — Camera 3-D → workspace frame (cm)
        ws_x, ws_y = self._cam3d_to_workspace(X, Y)

        # Step 3 — Workspace frame → robot frame (cm)
        rx, ry = self._workspace_to_robot(ws_x, ws_y)

        return {
            "class":       det["class"],
            "confidence":  det["confidence"],
            "center_px":   [cx, cy],
            "depth_m":     depth_m,
            "cam_3d_m":    {"X": round(X, 5),
                            "Y": round(Y, 5),
                            "Z": round(Z, 5)},
            "workspace":   {"x_cm": round(ws_x, 3),
                            "y_cm": round(ws_y, 3)},
            "robot":       {"x_cm": round(rx, 3),
                            "y_cm": round(ry, 3)},
        }

    # ── Static math helpers ────────────────────────────────────────────────────

    @staticmethod
    def _pixel_to_cam3d(cx: int, cy: int, depth_m: float,
                         fx: float, fy: float,
                         ppx: float, ppy: float) -> tuple[float, float, float]:
        
        X = (cx - ppx) * depth_m / fx
        Y = (cy - ppy) * depth_m / fy
        return X, Y, depth_m

    @staticmethod
    def _cam3d_to_workspace(X_m: float, Y_m: float) -> tuple[float, float]:
        
        ws_x = X_m * 100.0 + WORKSPACE_W_CM / 2.0
        ws_y = Y_m * 100.0 + WORKSPACE_H_CM / 2.0
        ws_x = float(np.clip(ws_x, 0.0, WORKSPACE_W_CM))
        ws_y = float(np.clip(ws_y, 0.0, WORKSPACE_H_CM))
        return ws_x, ws_y

    @staticmethod
    def _workspace_to_robot(ws_x: float, ws_y: float) -> tuple[float, float]:
        
        robot_x = -(ws_x - WORKSPACE_W_CM / 2.0)
        robot_y = -(ws_y - WORKSPACE_H_CM / 2.0)
        return robot_x, robot_y

    # ── Logging ────────────────────────────────────────────────────────────────

    def _log_result(self, r: dict):
        ws = r["workspace"]
        rb = r["robot"]
        self.get_logger().info(
            f"[{r['class']}  conf={r['confidence']:.2f}]  "
            f"px=({r['center_px'][0]}, {r['center_px'][1]})  "
            f"depth={r['depth_m']:.3f}m  |  "
            f"workspace=({ws['x_cm']:.2f}, {ws['y_cm']:.2f}) cm  |  "
            f"robot=({rb['x_cm']:.2f}, {rb['y_cm']:.2f}) cm"
        )


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    detector  = BottleCapDetectorNode()
    localizer = WorkspaceLocalizerNode()

    # MultiThreadedExecutor lets both nodes spin concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(localizer)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        localizer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()