import rclpy
from rclpy.node import Node
import struct
import os
from std_msgs.msg import Float32MultiArray

class ODriveCANNode(Node):
    def __init__(self):
        super().__init__('odrive_can_node')
        # Subscribe to topic /odrive/position_cmd
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/odrive/position_cmd',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Expect msg.data = [node_id, position]
        if len(msg.data) < 2:
            self.get_logger().error("Message must contain [node_id, position]")
            return

        node_id = int(msg.data[0])
        position = float(msg.data[1])
        velocity_ff = 0.0
        torque_ff = 0.0

        # Use fixed arbitration IDs for node 1 and 2
        if node_id == 1:
            arb_id = 0x02C
        elif node_id == 2:
            arb_id = 0x04C
        else:
            self.get_logger().error(f"Unsupported node_id {node_id}. Only 1 or 2 allowed.")
            return

        # Pack floats into 12 bytes (position, velocity_ff, torque_ff)
        data = struct.pack('<fff', position, velocity_ff, torque_ff)
        hex_data = data.hex()

        # Build cansend command
        cmd = f"cansend can0 {arb_id:03X}#{hex_data}"
        self.get_logger().info(f"Sending: {cmd}")
        os.system(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ODriveCANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    



##ros2 topic pub /odrive/position_cmd std_msgs/Float32MultiArray "{data: [1, 2.5]}"










