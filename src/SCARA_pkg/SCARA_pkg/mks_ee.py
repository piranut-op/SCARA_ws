"""MKS SERVO42D/57D CAN bridge for the SCARA prismatic ee_joint.

Subscribes to /mks/ee_cmd (std_msgs/Float32) carrying the ee_joint position
in METERS (matches the URDF), and emits an MKS F5 "absolute motion based
on coordinate values" frame to drive the stepper.

Frame format (Part 4 + §11.4.1, DLC=8, standard CAN ID = motor's slave ID):
    byte0 : 0xF5 (command code)
    byte1 : speed high  (uint16 big-endian, 0..3000 RPM)
    byte2 : speed low
    byte3 : acceleration (uint8, 0..255)
    byte4 : abs_axis [23:16]  (int24 big-endian, encoder counts)
    byte5 : abs_axis [15:8]
    byte6 : abs_axis  [7:0]
    byte7 : checksum   (sum of can_id + byte0..byte6) & 0xFF

Encoder convention (§5.1.2):
    1 turn CW  → encoder value += 0x4000
    1 turn CCW → encoder value -= 0x4000

Mechanical convention from project:
    URDF ee_joint axis is (0,0,-1) → positive ee_joint = world -Z (down).
    Stepper must rotate CCW 6 turns to translate the shaft 35 mm down.
    => +0.035 m of ee_joint  ↔  motor target = -98304 encoder counts
       (default direction_sign = -1).

Optional startup actions:
    - set working mode to 5 (Bus FOC) via 0x82
    - set current position as zero point via 0x92
"""

import math
import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


COUNTS_PER_REV = 16384       # MKS 14-bit encoder, 0x4000 per revolution
INT24_MIN, INT24_MAX = -8388607, 8388607
ERR_LOG_PERIOD_S = 2.0       # rate-limit CAN-error logging


def crc8_sum(can_id: int, payload: bytes) -> int:
    return (can_id + sum(payload)) & 0xFF


def encode_int24_be(v: int) -> bytes:
    if v < 0:
        v += 1 << 24  # two's complement in 24 bits
    return bytes([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


class MKSEndEffectorNode(Node):
    def __init__(self):
        super().__init__('mks_ee_can_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('mks_node_id', 3)

        # Mechanical mapping: 6 motor revolutions → 35 mm of shaft travel.
        self.declare_parameter('motor_revs_per_35mm', 6.0)
        self.declare_parameter('stroke_meters', 0.035)
        # Sign that maps URDF +ee_joint (down) to motor encoder counts.
        # CCW = negative encoder value (§5.1.2), and we want CCW for "down",
        # so the default is -1.
        self.declare_parameter('direction_sign', -1)

        # Software travel limits in METERS (matches URDF ee_joint range).
        self.declare_parameter('ee_min_m', -0.045)
        self.declare_parameter('ee_max_m',  0.065)

        # Motion profile (per F5 frame).
        self.declare_parameter('speed_rpm', 300)        # 0..3000
        self.declare_parameter('acceleration', 2)       # 0..255

        # Only re-issue F5 when target changes by more than this (meters).
        self.declare_parameter('deadband_m', 1.0e-4)

        # Optional one-shot startup commands.
        self.declare_parameter('set_bus_foc_on_start', False)
        self.declare_parameter('zero_on_start', False)

        gp = lambda n: self.get_parameter(n).value
        self.iface       = gp('can_interface')
        self.node_id     = int(gp('mks_node_id'))
        self.sign        = int(gp('direction_sign'))
        self.ee_min      = float(gp('ee_min_m'))
        self.ee_max      = float(gp('ee_max_m'))
        self.speed       = int(gp('speed_rpm'))
        self.accel       = int(gp('acceleration'))
        self.deadband    = float(gp('deadband_m'))

        revs_per_stroke  = float(gp('motor_revs_per_35mm'))
        stroke           = float(gp('stroke_meters'))
        # counts per meter (unsigned magnitude):
        self.counts_per_m = revs_per_stroke * COUNTS_PER_REV / stroke

        self.get_logger().info(
            f"ee_joint mapping: ±1 m → {int(self.counts_per_m)} counts "
            f"(sign={self.sign:+d}); "
            f"limits [{self.ee_min:+.3f}, {self.ee_max:+.3f}] m")

        # ── CAN bus ───────────────────────────────────────────────────────
        try:
            self.bus = can.interface.Bus(channel=self.iface,
                                         bustype='socketcan')
        except OSError as e:
            self.get_logger().fatal(
                f"Cannot open CAN '{self.iface}': {e}. "
                f"Bring it up: "
                f"`sudo ip link set {self.iface} up type can bitrate 250000`.")
            raise

        self._tx_lock = threading.Lock()
        self._last_meters = None
        self._tx_err_count = 0
        self._last_err_log = 0.0

        # ── Subscribers / Services-as-topics ──────────────────────────────
        self.create_subscription(Float32, '/mks/ee_cmd', self.on_ee_cmd, 10)
        # Trigger zero-set at any time by publishing True on this topic.
        self.create_subscription(Bool, '/mks/ee_set_zero',
                                 lambda m: self.send_set_zero() if m.data else None,
                                 10)

        # ── Optional startup ──────────────────────────────────────────────
        if bool(gp('set_bus_foc_on_start')):
            self.send_set_working_mode(5)  # 5 = Bus FOC
        if bool(gp('zero_on_start')):
            self.send_set_zero()

    # ── Helpers ───────────────────────────────────────────────────────────
    def _send_frame(self, payload: bytes, label: str):
        crc = crc8_sum(self.node_id, payload)
        data = payload + bytes([crc])
        msg = can.Message(arbitration_id=self.node_id,
                          data=data, is_extended_id=False)
        with self._tx_lock:
            try:
                self.bus.send(msg, timeout=0.0)
            except can.CanError as e:
                self._tx_err_count += 1
                now = time.monotonic()
                if now - self._last_err_log >= ERR_LOG_PERIOD_S:
                    self.get_logger().error(
                        f"[{label}] CAN send failed "
                        f"({self._tx_err_count} errors in last "
                        f"{ERR_LOG_PERIOD_S:.1f}s): {e}. "
                        f"Bus likely ERROR-PASSIVE — check motor / "
                        f"termination / power.")
                    self._tx_err_count = 0
                    self._last_err_log = now
                return
        self.get_logger().info(
            f"[{label}] id=0x{self.node_id:03X} data={data.hex()}")

    # ── Commands ──────────────────────────────────────────────────────────
    def send_set_working_mode(self, mode: int):
        self._send_frame(bytes([0x82, mode & 0xFF]),
                         f"set_mode={mode}")

    def send_set_zero(self):
        self._send_frame(bytes([0x92]), "set_zero")

    def send_emergency_stop(self):
        self._send_frame(bytes([0xF7]), "estop")

    def send_abs_target(self, ee_meters: float):
        # Clamp to URDF range.
        clamped = max(self.ee_min, min(self.ee_max, ee_meters))
        if clamped != ee_meters:
            self.get_logger().warn(
                f"ee_cmd {ee_meters:+.4f} m clamped to {clamped:+.4f} m.")

        counts = int(round(self.sign * clamped * self.counts_per_m))
        if not (INT24_MIN <= counts <= INT24_MAX):
            self.get_logger().error(
                f"Computed counts {counts} out of int24 range; aborting.")
            return

        speed_be = struct.pack('>H', max(0, min(3000, self.speed)))
        payload = bytes([0xF5]) + speed_be + bytes([self.accel]) \
            + encode_int24_be(counts)
        self._send_frame(
            payload,
            f"abs ee={clamped:+.4f}m → counts={counts:+d} "
            f"(speed={self.speed} rpm, acc={self.accel})")

    # ── Subscribers ───────────────────────────────────────────────────────
    def on_ee_cmd(self, msg: Float32):
        target = float(msg.data)
        if (self._last_meters is not None
                and abs(target - self._last_meters) < self.deadband):
            return
        self._last_meters = target
        self.send_abs_target(target)

    # ── Cleanup ───────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self.bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MKSEndEffectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
