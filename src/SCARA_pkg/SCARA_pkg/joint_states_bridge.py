"""Quick path: tap MoveIt2 fake-hardware /joint_states and republish to the
SCARA hardware command topics.

  /joint_states  ──▶  /odrive/angle_cmd   (ODrive joints 1, 2)
                ──▶  /mks/ee_cmd          (MKS prismatic ee_joint, meters)

This lets the simulation in ~/scara_bot_ws/ drive the real robot without
touching ros2_control. Both downstream nodes (newposition, mks_ee) handle
their own CAN bus and unit conversion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray


class JointStatesBridge(Node):
    def __init__(self):
        super().__init__('joint_states_bridge')

        self.declare_parameter('joint1_name', 'Link_1_joint')
        self.declare_parameter('joint2_name', 'Link_2_joint')
        self.declare_parameter('ee_joint_name', 'ee_joint')
        self.declare_parameter('joint1_node_id', 1)
        self.declare_parameter('joint2_node_id', 2)
        # Republish-on-change thresholds so we don't flood the CAN bus.
        self.declare_parameter('arm_deadband_rad', 1.0e-3)
        self.declare_parameter('ee_deadband_m',    1.0e-4)
        # Hard rate cap so we never spam the CAN bus if joint_states
        # publishes at hundreds of Hz.
        self.declare_parameter('max_publish_hz', 50.0)

        gp = lambda n: self.get_parameter(n).value
        self.j1_name = gp('joint1_name')
        self.j2_name = gp('joint2_name')
        self.ee_name = gp('ee_joint_name')
        self.j1_node = int(gp('joint1_node_id'))
        self.j2_node = int(gp('joint2_node_id'))
        self.arm_deadband = float(gp('arm_deadband_rad'))
        self.ee_deadband  = float(gp('ee_deadband_m'))
        max_hz = max(1.0, float(gp('max_publish_hz')))
        self._min_period = 1.0 / max_hz

        self.pub_arm = self.create_publisher(
            Float32MultiArray, '/odrive/angle_cmd', 10)
        self.pub_ee  = self.create_publisher(
            Float32, '/mks/ee_cmd', 10)

        self.create_subscription(JointState, '/joint_states',
                                 self.on_joint_states, 50)

        self._last_j1 = None
        self._last_j2 = None
        self._last_ee = None
        self._last_arm_pub_t = 0.0
        self._last_ee_pub_t = 0.0

        self.get_logger().info(
            f"Bridging /joint_states → /odrive/angle_cmd "
            f"({self.j1_name}=node {self.j1_node}, "
            f"{self.j2_name}=node {self.j2_node}) "
            f"and /mks/ee_cmd ({self.ee_name}, meters).")

    def on_joint_states(self, msg: JointState):
        import time
        now = time.monotonic()

        pos_by_name = dict(zip(msg.name, msg.position))
        j1 = pos_by_name.get(self.j1_name)
        j2 = pos_by_name.get(self.j2_name)
        ee = pos_by_name.get(self.ee_name)

        if j1 is not None and j2 is not None \
                and (now - self._last_arm_pub_t) >= self._min_period:
            moved = (
                self._last_j1 is None or self._last_j2 is None
                or abs(j1 - self._last_j1) >= self.arm_deadband
                or abs(j2 - self._last_j2) >= self.arm_deadband
            )
            if moved:
                arm = Float32MultiArray()
                arm.data = [float(self.j1_node), float(j1),
                            float(self.j2_node), float(j2)]
                self.pub_arm.publish(arm)
                self._last_j1, self._last_j2 = j1, j2
                self._last_arm_pub_t = now

        if ee is not None \
                and (now - self._last_ee_pub_t) >= self._min_period:
            if (self._last_ee is None
                    or abs(ee - self._last_ee) >= self.ee_deadband):
                self.pub_ee.publish(Float32(data=float(ee)))
                self._last_ee = ee
                self._last_ee_pub_t = now


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
