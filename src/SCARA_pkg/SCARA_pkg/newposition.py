"""ODrive joint-angle → CAN bridge for the SCARA's two revolute joints.

Subscribes to /odrive/angle_cmd (Float32MultiArray) with payload
[node1_id, angle1_rad, node2_id, angle2_rad] and emits one
Set_Input_Pos (cmd 0x0C) frame per joint over socketcan.

Frame format (ODrive 0.5.x, Set_Input_Pos, 8 bytes):
    bytes 0..3 : float32 position   (turns)
    bytes 4..5 : int16  vel_ff      (turns/s × 1000)
    bytes 6..7 : int16  torque_ff   (Nm × 1000)
Arbitration ID = (node_id << 5) | 0x0C, 11-bit standard frame.
"""

import math
import struct
import time

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


SET_INPUT_POS = 0x0C       # ODrive command ID
ERR_LOG_PERIOD_S = 2.0     # rate-limit CAN-error logging


class JointMap:
    """Linear map from joint angle (rad) to motor position (turns).

    direction_sign: +1 if motor rotates the same way as the URDF joint axis,
    -1 if mounted such that positive joint angle requires negative motor
    turns (e.g. joint 2 mounted upside-down).
    """

    def __init__(self, gear_ratio: float, offset_turns: float,
                 angle_min: float, angle_max: float,
                 direction_sign: int = 1):
        self.k = direction_sign * gear_ratio / (2.0 * math.pi)  # turns/rad
        self.offset = offset_turns
        self.angle_min = angle_min
        self.angle_max = angle_max

    def angle_to_turns(self, angle_rad: float) -> float:
        return self.k * angle_rad + self.offset

    def clamp(self, angle_rad: float):
        clamped = max(self.angle_min, min(self.angle_max, angle_rad))
        return clamped, clamped != angle_rad


class ODriveAngleCANNode(Node):
    def __init__(self):
        super().__init__('odrive_angle_can_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('joint1_node_id', 1)
        self.declare_parameter('joint2_node_id', 2)
        self.declare_parameter('joint1_gear_ratio', 100.0)
        self.declare_parameter('joint2_gear_ratio', 100.0)
        # Motor turns at joint angle = 0 (home offset).
        # Both default to 0 so URDF joint=0 maps to whatever motor position
        # the ODrive was zeroed at during bring-up. The robot does NOT move
        # at startup as long as the hardware was homed before the bridge ran.
        self.declare_parameter('joint1_offset_turns', 0.0)
        self.declare_parameter('joint2_offset_turns', 0.0)
        # +1 = motor & URDF axis agree; -1 = motor flipped (upside-down mount).
        # Joint 2 defaults to -1 because the second SCARA stage hangs
        # downward, inverting its rotation direction relative to the URDF.
        self.declare_parameter('joint1_direction_sign',  1)
        self.declare_parameter('joint2_direction_sign', -1)
        # Software joint limits (rad). Match the URDF Link_*_joint limits
        # so a runaway command can't drive the BLDC past mechanical stops.
        self.declare_parameter('joint1_min_rad', -1.5)
        self.declare_parameter('joint1_max_rad',  1.6)
        self.declare_parameter('joint2_min_rad', -2.3)
        self.declare_parameter('joint2_max_rad',  2.7)

        gp = lambda n: self.get_parameter(n).value
        self.iface = gp('can_interface')
        self.joint_maps = {
            int(gp('joint1_node_id')): JointMap(
                gp('joint1_gear_ratio'), gp('joint1_offset_turns'),
                gp('joint1_min_rad'),    gp('joint1_max_rad'),
                int(gp('joint1_direction_sign'))),
            int(gp('joint2_node_id')): JointMap(
                gp('joint2_gear_ratio'), gp('joint2_offset_turns'),
                gp('joint2_min_rad'),    gp('joint2_max_rad'),
                int(gp('joint2_direction_sign'))),
        }

        # ── CAN bus ───────────────────────────────────────────────────────
        try:
            self.bus = can.interface.Bus(channel=self.iface,
                                         bustype='socketcan')
        except OSError as e:
            self.get_logger().fatal(
                f"Cannot open CAN interface '{self.iface}': {e}. "
                f"Bring it up first: "
                f"`sudo ip link set {self.iface} up type can bitrate 250000`.")
            raise

        self.get_logger().info(
            f"ODrive angle→CAN bridge ready on {self.iface}, "
            f"nodes={list(self.joint_maps.keys())}")

        self._tx_err_count = 0
        self._last_err_log = 0.0

        # ── Subscriber ────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Float32MultiArray, '/odrive/angle_cmd', self.on_angle_cmd, 10)

    # ── Callbacks ─────────────────────────────────────────────────────────
    def on_angle_cmd(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().error(
                "Expected [node1_id, angle1_rad, node2_id, angle2_rad].")
            return

        for node_id, angle_rad in (
            (int(msg.data[0]), float(msg.data[1])),
            (int(msg.data[2]), float(msg.data[3])),
        ):
            self.send_set_input_pos(node_id, angle_rad)

    # ── ODrive frame builder ──────────────────────────────────────────────
    def send_set_input_pos(self, node_id: int, angle_rad: float,
                           vel_ff_turns_s: float = 0.0,
                           torque_ff_nm: float = 0.0):
        jmap = self.joint_maps.get(node_id)
        if jmap is None:
            self.get_logger().error(
                f"Unknown node_id {node_id}; configured: "
                f"{list(self.joint_maps.keys())}")
            return

        clamped, was_clamped = jmap.clamp(angle_rad)
        if was_clamped:
            self.get_logger().warn(
                f"Node {node_id}: angle {angle_rad:.3f} rad clamped to "
                f"{clamped:.3f} rad (limits "
                f"[{jmap.angle_min:.3f}, {jmap.angle_max:.3f}]).")

        position_turns = jmap.angle_to_turns(clamped)

        # Pack 8-byte payload: float32 pos | int16 vel_ff×1000 | int16 trq_ff×1000
        vel_ff_i = int(round(vel_ff_turns_s * 1000.0))
        trq_ff_i = int(round(torque_ff_nm   * 1000.0))
        data = struct.pack('<fhh', position_turns, vel_ff_i, trq_ff_i)

        arb_id = (node_id << 5) | SET_INPUT_POS
        msg = can.Message(arbitration_id=arb_id, data=data,
                          is_extended_id=False)
        try:
            self.bus.send(msg, timeout=0.0)  # non-blocking; let qdisc reject
        except can.CanError as e:
            self._tx_err_count += 1
            now = time.monotonic()
            if now - self._last_err_log >= ERR_LOG_PERIOD_S:
                self.get_logger().error(
                    f"CAN send failed (node {node_id}, "
                    f"{self._tx_err_count} errors in last "
                    f"{ERR_LOG_PERIOD_S:.1f}s): {e}. "
                    f"Bus likely ERROR-PASSIVE — check motors / "
                    f"termination / power.")
                self._tx_err_count = 0
                self._last_err_log = now
            return

        self.get_logger().debug(
            f"node={node_id} angle={clamped:+.3f} rad → pos={position_turns:+.3f} "
            f"turns | arb=0x{arb_id:03X} data={data.hex()}")

    # ── Cleanup ───────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self.bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ODriveAngleCANNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
