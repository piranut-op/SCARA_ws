#!/usr/bin/env python3
"""Bridge bottle-cap detections to the SCARA IK target.

Subscribes:
  /bottle_cap/workspace_position  (std_msgs/String, JSON list)

Publishes:
  /ik_target  (std_msgs/Float32MultiArray, [x_m, y_m] in SCARA base frame)

The detector reports `robot.x_cm` / `robot.y_cm` with the origin at the
workspace centre (camera-derived). The SCARA IK expects coordinates in its
base frame (origin at joint 1, y forward and strictly positive). We convert
cm -> m and apply a calibrated offset from the workspace centre to the
SCARA base origin.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class CapToTargetNode(Node):
    def __init__(self):
        super().__init__("cap_to_target")

        self.declare_parameter("class_filter", "bottle_cap")
        self.declare_parameter("min_confidence", 0.40)
        self.declare_parameter("base_to_workspace_x_m", 0.0)
        self.declare_parameter("base_to_workspace_y_m", 0.20)
        self.declare_parameter("deadband_m", 0.005)
        self.declare_parameter("min_publish_period_s", 0.20)

        self._class_filter   = self.get_parameter("class_filter").value
        self._min_conf       = float(self.get_parameter("min_confidence").value)
        self._off_x          = float(self.get_parameter("base_to_workspace_x_m").value)
        self._off_y          = float(self.get_parameter("base_to_workspace_y_m").value)
        self._deadband       = float(self.get_parameter("deadband_m").value)
        self._min_period     = float(self.get_parameter("min_publish_period_s").value)

        self._sub = self.create_subscription(
            String, "/bottle_cap/workspace_position", self._on_localized, 10
        )
        self._pub = self.create_publisher(Float32MultiArray, "/ik_target", 10)

        self._last_xy = None
        self._last_pub_t = 0.0

        self.get_logger().info(
            f"cap_to_target ready  |  class='{self._class_filter}'  "
            f"min_conf={self._min_conf:.2f}  "
            f"offset=({self._off_x:.3f}, {self._off_y:.3f}) m  "
            f"deadband={self._deadband*1000:.1f} mm  "
            f"min_period={self._min_period*1000:.0f} ms"
        )

    def _on_localized(self, msg: String):
        try:
            results = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Malformed localizer JSON — skipping.")
            return
        if not results:
            return

        candidates = [
            r for r in results
            if (not self._class_filter or r.get("class") == self._class_filter)
            and float(r.get("confidence", 0.0)) >= self._min_conf
        ]
        if not candidates:
            return

        best = max(candidates, key=lambda r: float(r["confidence"]))
        rx_cm = float(best["robot"]["x_cm"])
        ry_cm = float(best["robot"]["y_cm"])

        x_m = rx_cm / 100.0 + self._off_x
        y_m = ry_cm / 100.0 + self._off_y

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_xy is not None:
            dx = x_m - self._last_xy[0]
            dy = y_m - self._last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self._deadband:
                return
            if (now - self._last_pub_t) < self._min_period:
                return

        out = Float32MultiArray()
        out.data = [float(x_m), float(y_m)]
        self._pub.publish(out)
        self._last_xy = (x_m, y_m)
        self._last_pub_t = now

        self.get_logger().info(
            f"[{best['class']} conf={float(best['confidence']):.2f}]  "
            f"workspace_robot=({rx_cm:.2f},{ry_cm:.2f}) cm  ->  "
            f"/ik_target=({x_m:.3f},{y_m:.3f}) m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CapToTargetNode()
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
