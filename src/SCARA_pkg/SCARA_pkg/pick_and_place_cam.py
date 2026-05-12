#!/usr/bin/env python3
"""Camera-driven SCARA pick-and-place state machine.

Pipeline this node sits on top of:

    detect_bottle_cap  -> /bottle_cap/detections (geometry_msgs/PointStamped,
                          a unit-Z ray in camera_color_optical_frame)
    ikpos              <- /ik_target             -> /odrive/angle_cmd
    mks_ee             <- /mks/ee_cmd            (prismatic, metres)

The detector publishes pixel rays (no depth — RealSense IR depth is
unreliable on matte black workspaces). This node uses tf2 to transform the
ray into `base_link` and intersects it with the workspace plane at
`workspace_z_m` to get the cap's (bx, by). The result is shifted by
`base_to_workspace_x_m / y_m` (in case the downstream IK uses a different
frame, e.g. shoulder-relative) and fed to /ik_target.

States:
  IDLE          waiting for a stable cap detection
  STABILIZE     tracking the same cap for N consecutive frames within deadband
  APPROACH      send /ik_target above the cap, EE up
  DESCEND_PICK  EE down to pick height
  DWELL_PICK    hold (passive vacuum/magnet engages)
  LIFT_PICK     EE up to travel height
  TO_PLACE      /ik_target = drop xy
  DESCEND_PLACE EE down to drop height
  DWELL_PLACE   hold (passive release)
  LIFT_PLACE    EE up
  HOME          /ik_target = home xy, EE up
  COOLDOWN      ignore detections briefly to avoid retriggering

The EE is treated as a passive end-effector: descending engages the cap,
lifting carries it. If you have a discrete gripper trigger, set
`gripper_topic` to publish a Bool there at pick/release moments (TODO: wire
up — currently only logs).
"""

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PointStamped for tf2.transform


class S(Enum):
    IDLE          = auto()
    STABILIZE     = auto()
    APPROACH      = auto()
    DESCEND_PICK  = auto()
    DWELL_PICK    = auto()
    LIFT_PICK     = auto()
    TO_PLACE      = auto()
    DESCEND_PLACE = auto()
    DWELL_PLACE   = auto()
    LIFT_PLACE    = auto()
    HOME          = auto()
    COOLDOWN      = auto()


class PickAndPlaceCamNode(Node):
    def __init__(self):
        super().__init__("pick_and_place_cam")

        # Detection filtering — kept as parameters for backwards compat,
        # but the detector node already filters by class + confidence and
        # gates on multi-second pixel stability before publishing, so these
        # are effectively unused here.
        self.declare_parameter("class_filter",   "bottle_cap")
        self.declare_parameter("min_confidence", 0.50)

        # Workspace-plane height in base_link (metres). The cap is assumed
        # to sit on this plane; the ray from the camera is intersected with
        # it to get base_link (x, y).
        self.declare_parameter("workspace_z_m", 0.0)

        # If the IK frame differs from base_link (e.g. shoulder-relative),
        # this offset is added to the base_link (bx, by) before /ik_target
        # is published. Set to (0, 0) to pass base_link through directly.
        self.declare_parameter("base_to_workspace_x_m", 0.0)
        self.declare_parameter("base_to_workspace_y_m", 0.0)

        # Stability gate: cap must persist N frames within deadband
        self.declare_parameter("stable_frames",     8)
        self.declare_parameter("stable_deadband_m", 0.01)

        # EE heights (metres in mks_ee frame; negative = down with default sign)
        self.declare_parameter("ee_travel_m",  0.060)   # safe travel height
        self.declare_parameter("ee_pick_m",   -0.040)   # touch-the-cap height
        self.declare_parameter("ee_place_m",  -0.040)

        # Drop / home positions (SCARA base frame, metres)
        self.declare_parameter("place_x_m", 0.10)
        self.declare_parameter("place_y_m", 0.25)
        self.declare_parameter("home_x_m",  0.0)
        self.declare_parameter("home_y_m",  0.20)

        # Timing (seconds) — tune to your motion profile
        self.declare_parameter("approach_s",     2.0)
        self.declare_parameter("descend_s",      1.5)
        self.declare_parameter("dwell_pick_s",   0.8)
        self.declare_parameter("lift_s",         1.5)
        self.declare_parameter("to_place_s",     2.5)
        self.declare_parameter("dwell_place_s",  0.8)
        self.declare_parameter("home_s",         2.0)
        self.declare_parameter("cooldown_s",     2.0)

        # Optional discrete gripper trigger (Bool); empty => disabled
        self.declare_parameter("gripper_topic", "")

        gp = lambda n: self.get_parameter(n).value
        self._class_filter   = gp("class_filter")
        self._min_conf       = float(gp("min_confidence"))
        self._workspace_z    = float(gp("workspace_z_m"))
        self._off_x          = float(gp("base_to_workspace_x_m"))
        self._off_y          = float(gp("base_to_workspace_y_m"))
        self._stable_n       = int(gp("stable_frames"))
        self._stable_db      = float(gp("stable_deadband_m"))
        self._ee_travel      = float(gp("ee_travel_m"))
        self._ee_pick        = float(gp("ee_pick_m"))
        self._ee_place       = float(gp("ee_place_m"))
        self._place_xy       = (float(gp("place_x_m")), float(gp("place_y_m")))
        self._home_xy        = (float(gp("home_x_m")),  float(gp("home_y_m")))

        self._t_approach     = float(gp("approach_s"))
        self._t_descend      = float(gp("descend_s"))
        self._t_dwell_pick   = float(gp("dwell_pick_s"))
        self._t_lift         = float(gp("lift_s"))
        self._t_to_place     = float(gp("to_place_s"))
        self._t_dwell_place  = float(gp("dwell_place_s"))
        self._t_home         = float(gp("home_s"))
        self._t_cooldown     = float(gp("cooldown_s"))

        gripper_topic = gp("gripper_topic")
        self._gripper_pub = (
            self.create_publisher(Bool, gripper_topic, 1) if gripper_topic else None
        )

        # tf2 — detector publishes points in camera_color_optical_frame;
        # the launch file (or another node) must publish a static TF from
        # base_link to that frame.
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            PointStamped, "/bottle_cap/detections", self._on_detection, 10
        )
        self._ik_pub = self.create_publisher(Float32MultiArray, "/ik_target", 10)
        self._ee_pub = self.create_publisher(Float32, "/mks/ee_cmd", 10)

        self._state = S.IDLE
        self._state_t0 = self.get_clock().now()
        self._stable_count = 0
        self._stable_xy = None
        self._target_xy = None

        # 10 Hz tick to advance time-driven states
        self.create_timer(0.1, self._tick)

        # Park EE up immediately so we start in a known pose
        self._send_ee(self._ee_travel)
        self.get_logger().info(
            f"pick_and_place_cam ready  |  class='{self._class_filter}'  "
            f"offset=({self._off_x:.3f},{self._off_y:.3f}) m  "
            f"place={self._place_xy}  home={self._home_xy}"
        )

    # ── helpers ────────────────────────────────────────────────────────────

    def _send_ik(self, xy):
        msg = Float32MultiArray()
        msg.data = [float(xy[0]), float(xy[1])]
        self._ik_pub.publish(msg)

    def _send_ee(self, z_m):
        msg = Float32()
        msg.data = float(z_m)
        self._ee_pub.publish(msg)

    def _send_gripper(self, closed: bool):
        if self._gripper_pub:
            self._gripper_pub.publish(Bool(data=closed))
        self.get_logger().info(f"gripper -> {'CLOSE' if closed else 'OPEN'}")

    def _enter(self, state: S):
        self.get_logger().info(f"  state: {self._state.name} -> {state.name}")
        self._state = state
        self._state_t0 = self.get_clock().now()

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_t0).nanoseconds * 1e-9

    # ── detection callback (only consumed in IDLE/STABILIZE) ───────────────

    def _project_ray_to_base(self, msg: PointStamped):
        """Transform two points from the camera optical frame into base_link
        (the camera origin and the unit-Z point along the pixel ray) and
        intersect with z = workspace_z_m. Returns (bx, by) or None on
        failure."""
        origin_in_cam = PointStamped()
        origin_in_cam.header = msg.header
        origin_in_cam.point.x = 0.0
        origin_in_cam.point.y = 0.0
        origin_in_cam.point.z = 0.0

        end_in_cam = PointStamped()
        end_in_cam.header = msg.header
        end_in_cam.point  = msg.point

        try:
            origin_b = self._tf_buffer.transform(
                origin_in_cam, "base_link", timeout=rclpy.duration.Duration(seconds=0.5))
            end_b = self._tf_buffer.transform(
                end_in_cam, "base_link", timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as ex:
            self.get_logger().warn(f"TF transform failed: {ex}")
            return None

        dx = end_b.point.x - origin_b.point.x
        dy = end_b.point.y - origin_b.point.y
        dz = end_b.point.z - origin_b.point.z
        if abs(dz) < 1e-6:
            self.get_logger().warn(
                f"Ray nearly parallel to workspace plane (dz={dz:.6f}); skipping.")
            return None
        t = (self._workspace_z - origin_b.point.z) / dz
        if t <= 0.0:
            self.get_logger().warn(
                f"Ray points away from workspace (t={t:.3f}); skipping.")
            return None
        bx = origin_b.point.x + t * dx
        by = origin_b.point.y + t * dy
        return (bx, by)

    def _on_detection(self, msg: PointStamped):
        if self._state not in (S.IDLE, S.STABILIZE):
            return

        proj = self._project_ray_to_base(msg)
        if proj is None:
            return
        bx, by = proj

        # Apply optional shift in case /ik_target is in a different frame
        # (e.g. shoulder-relative). Defaults to (0, 0) — pass-through.
        x_m = bx + self._off_x
        y_m = by + self._off_y

        self.get_logger().info(
            f"ray=({msg.point.x:.4f},{msg.point.y:.4f},1.0) "
            f"→ base_link=({bx:.3f},{by:.3f}) "
            f"→ /ik_target=({x_m:.3f},{y_m:.3f})"
        )

        if self._stable_xy is None:
            self._stable_xy = (x_m, y_m)
            self._stable_count = 1
            self._enter(S.STABILIZE)
            return

        dx = x_m - self._stable_xy[0]
        dy = y_m - self._stable_xy[1]
        if (dx * dx + dy * dy) ** 0.5 <= self._stable_db:
            self._stable_count += 1
            self._stable_xy = (
                0.5 * (self._stable_xy[0] + x_m),
                0.5 * (self._stable_xy[1] + y_m),
            )
        else:
            self._stable_xy = (x_m, y_m)
            self._stable_count = 1

        if self._stable_count >= self._stable_n:
            self._target_xy = self._stable_xy
            self._stable_count = 0
            self._stable_xy = None
            self.get_logger().info(
                f"cap stable at ({self._target_xy[0]:.3f}, "
                f"{self._target_xy[1]:.3f}) m — starting pick."
            )
            self._send_ee(self._ee_travel)
            self._send_ik(self._target_xy)
            self._enter(S.APPROACH)

    # ── timed state advancer ───────────────────────────────────────────────

    def _tick(self):
        s = self._state
        e = self._elapsed()

        if   s == S.APPROACH and e >= self._t_approach:
            self._send_ee(self._ee_pick)
            self._enter(S.DESCEND_PICK)

        elif s == S.DESCEND_PICK and e >= self._t_descend:
            self._send_gripper(True)
            self._enter(S.DWELL_PICK)

        elif s == S.DWELL_PICK and e >= self._t_dwell_pick:
            self._send_ee(self._ee_travel)
            self._enter(S.LIFT_PICK)

        elif s == S.LIFT_PICK and e >= self._t_lift:
            self._send_ik(self._place_xy)
            self._enter(S.TO_PLACE)

        elif s == S.TO_PLACE and e >= self._t_to_place:
            self._send_ee(self._ee_place)
            self._enter(S.DESCEND_PLACE)

        elif s == S.DESCEND_PLACE and e >= self._t_descend:
            self._send_gripper(False)
            self._enter(S.DWELL_PLACE)

        elif s == S.DWELL_PLACE and e >= self._t_dwell_place:
            self._send_ee(self._ee_travel)
            self._enter(S.LIFT_PLACE)

        elif s == S.LIFT_PLACE and e >= self._t_lift:
            self._send_ik(self._home_xy)
            self._enter(S.HOME)

        elif s == S.HOME and e >= self._t_home:
            self._enter(S.COOLDOWN)

        elif s == S.COOLDOWN and e >= self._t_cooldown:
            self._enter(S.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceCamNode()
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
