# # import rclpy
# # from rclpy.node import Node
# # from std_msgs.msg import Float32MultiArray
# # import math

# import rclpy
# from rclpy.node import Node
# import struct
# import os
# from std_msgs.msg import Float32MultiArray
# import math

# class ODriveAngleCANNode(Node):
#     def __init__(self):
#         super().__init__('odrive_angle_can_node')

#         # Subscribe to topic /odrive/angle_cmd
#         # Expect msg.data = [node_id, angle_rad]
#         self.subscription = self.create_subscription(
#             Float32MultiArray,
#             '/odrive/angle_cmd',
#             self.listener_callback,
#             10)

#         # Conversion: 0 rad -> -25, pi rad -> +25
#         self.factor = 50.0 / math.pi
#         self.offset = -25.0

#     def listener_callback(self, msg):
#         if len(msg.data) < 2:
#             self.get_logger().error("Message must contain [node_id, angle_rad]")
#             return

#         node_id = int(msg.data[0])
#         angle_rad = float(msg.data[1])

#         # Convert angle to position
#         position = self.factor * angle_rad + self.offset
#         velocity_ff = 0.0
#         torque_ff = 0.0

#         # Arbitration IDs for node 1 and 2
#         if node_id == 1:
#             arb_id = 0x02C
#         elif node_id == 2:
#             arb_id = 0x04C
#         else:
#             self.get_logger().error(f"Unsupported node_id {node_id}. Only 1 or 2 allowed.")
#             return

#         # Pack floats into 12 bytes (position, velocity_ff, torque_ff)
#         data = struct.pack('<fff', position, velocity_ff, torque_ff)
#         hex_data = data.hex()

#         # Build cansend command
#         cmd = f"cansend can0 {arb_id:03X}#{hex_data}"
#         self.get_logger().info(
#             f"Node {node_id}: Angle {angle_rad:.3f} rad -> Position {position:.3f} | Sending: {cmd}"
#         )
#         os.system(cmd)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ODriveAngleCANNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




import rclpy
from rclpy.node import Node
import struct
import os
from std_msgs.msg import Float32MultiArray
import math

class ODriveAngleCANNode(Node):
    def __init__(self):
        super().__init__('odrive_angle_can_node')

        # Subscribe to topic /odrive/angle_cmd
        # Expect msg.data = [node1_id, angle1, node2_id, angle2]
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/odrive/angle_cmd',
            self.listener_callback,
            10)

        # Conversion: 0 rad -> -25, pi rad -> +25
        self.factor = 50.0 / math.pi
        self.offset = -25.0

    def listener_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().error("Message must contain [node1_id, angle1, node2_id, angle2]")
            return

        # Extract both motor commands
        node1_id = int(msg.data[0])
        angle1_rad = float(msg.data[1])
        node2_id = int(msg.data[2])
        angle2_rad = float(msg.data[3])

        # Send CAN commands for both motors
        self.send_can_command(node1_id, angle1_rad)
        self.send_can_command(node2_id, angle2_rad)

    def send_can_command(self, node_id, angle_rad):
        # Convert angle to position
        position = self.factor * angle_rad + self.offset
        velocity_ff = 0.0
        torque_ff = 0.0

        # Arbitration IDs for node 1 and 2
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
        self.get_logger().info(
            f"Node {node_id}: Angle {angle_rad:.3f} rad -> Position {position:.3f} | Sending: {cmd}"
        )
        os.system(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ODriveAngleCANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

