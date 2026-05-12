"""Analytical SCARA IK aligned with the URDF.

Geometry (from src/scara_robot/urdf/arm.xacro):
    Link_1_joint at base_link xy = (-0.1035, -0.0125), axis (0, 0, -1)
    Link_2_joint at Link_1   xy = ( 0.14912, 0.00021), axis (0, 0, +1)
    ee_joint origin in Link_2 xy = ( 0.14661, -0.03577)
    URDF joint limits:
        Link_1_joint: [-1.5, +1.6]
        Link_2_joint: [-2.3, +2.7]

Forward kinematics with q1 about -Z and q2 about +Z, plus the L2 phase
offset BETA = atan2(-0.035766, 0.14661):
    EE_x = SHOULDER_X + L1 cos(-q1) + L2 cos(-q1 + q2 + BETA)
    EE_y = SHOULDER_Y + L1 sin(-q1) + L2 sin(-q1 + q2 + BETA)

Solving (elbow-up branch, alpha negative):
    tx = x - SHOULDER_X,  ty = y - SHOULDER_Y
    cos_alpha = (tx² + ty² - L1² - L2²) / (2 L1 L2)
    alpha = -acos(cos_alpha)
    u     = atan2(ty, tx) - atan2(L2 sin α, L1 + L2 cos α)
    q1 = -u
    q2 = alpha - BETA

The /ik_target topic now expects (x, y) in `base_link` (metres). The
published angles go to /odrive/angle_cmd as [1.0, q1, 2.0, q2] — same
format newposition.py expects. If the hardware rotates the wrong way for
joint 1, set `invert_q1:=true` (it is purely a motor-wiring sign flip).
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# ── URDF geometry constants ───────────────────────────────────────────
# Link_1_joint origin in base_link: (0, 0, 0.16) — shoulder pivot is
# directly above base_link's origin on the mounting plate.
SHOULDER_X = 0.0
SHOULDER_Y = 0.0
L1         =  0.14912
L2_OFF_X   =  0.14661
L2_OFF_Y   = -0.035766
L2         = math.hypot(L2_OFF_X, L2_OFF_Y)            # ≈ 0.15090
BETA       = math.atan2(L2_OFF_Y, L2_OFF_X)            # ≈ -0.241 rad

# URDF joint limits
Q1_LO, Q1_HI = -1.5,  1.6
Q2_LO, Q2_HI = -2.3,  2.7


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__("inverse_kinematics_node")

        # Sign flip for joint 1 in case the motor is wired opposite to
        # URDF convention. Only effect is q1 -> -q1 on publish.
        self.declare_parameter("invert_q1", False)
        self.declare_parameter("invert_q2", False)
        # Elbow branch: True = elbow-up (alpha negative), False = elbow-down.
        self.declare_parameter("elbow_up", True)

        self._invert_q1 = bool(self.get_parameter("invert_q1").value)
        self._invert_q2 = bool(self.get_parameter("invert_q2").value)
        self._elbow_up  = bool(self.get_parameter("elbow_up").value)

        self.publisher_ = self.create_publisher(
            Float32MultiArray, "/odrive/angle_cmd", 10)

        self.subscription = self.create_subscription(
            Float32MultiArray, "/ik_target", self.listener_callback, 10)

        self.get_logger().info(
            f"IK ready  |  shoulder=({SHOULDER_X:.4f},{SHOULDER_Y:.4f})  "
            f"L1={L1:.4f}  L2={L2:.4f}  BETA={BETA:.4f}  "
            f"elbow_up={self._elbow_up}  invert_q1={self._invert_q1}  "
            f"invert_q2={self._invert_q2}"
        )

    def listener_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().error("IK target needs [x, y] in base_link")
            return

        x = float(msg.data[0])
        y = float(msg.data[1])

        tx = x - SHOULDER_X
        ty = y - SHOULDER_Y
        r2 = tx * tx + ty * ty
        r  = math.sqrt(r2)

        cos_alpha = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if cos_alpha < -1.0 or cos_alpha > 1.0:
            self.get_logger().warn(
                f"IK out of reach: target=({x:.3f},{y:.3f}) base_link, "
                f"r={r:.3f} from shoulder, max={L1 + L2:.3f}"
            )
            return

        alpha = math.acos(cos_alpha)
        if self._elbow_up:
            alpha = -alpha

        u  = math.atan2(ty, tx) - math.atan2(
            L2 * math.sin(alpha), L1 + L2 * math.cos(alpha))
        q1 = -u
        q2 = alpha - BETA

        if not (Q1_LO <= q1 <= Q1_HI and Q2_LO <= q2 <= Q2_HI):
            self.get_logger().warn(
                f"IK joint limits exceeded: q1={q1:.3f} (limit "
                f"[{Q1_LO},{Q1_HI}]) q2={q2:.3f} (limit [{Q2_LO},{Q2_HI}])"
            )
            return

        out_q1 = -q1 if self._invert_q1 else q1
        out_q2 = -q2 if self._invert_q2 else q2

        out_msg = Float32MultiArray()
        out_msg.data = [1.0, out_q1, 2.0, out_q2]
        self.publisher_.publish(out_msg)

        self.get_logger().info(
            f"IK target=({x:.3f},{y:.3f}) base_link -> "
            f"q1={q1:.4f} q2={q2:.4f} (published [{out_q1:.4f},{out_q2:.4f}])"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
