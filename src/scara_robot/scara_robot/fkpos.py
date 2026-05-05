# import rclpy
# from rclpy.node import Node
# import numpy as np
# from std_msgs.msg import Float32MultiArray
# import math

# class ForwardKinematicsNode(Node):
#     def __init__(self):
#         super().__init__('forward_kinematics_node')

#         # Subscribe to the same topic that carries angles
#         self.subscription = self.create_subscription(
#             Float32MultiArray,
#             '/odrive/angle_cmd',   # <-- same topic
#             self.listener_callback,
#             10)

#         # Publisher for end-effector coordinates
#         self.publisher = self.create_publisher(Float32MultiArray, '/end_effector_xy', 10)

#     def listener_callback(self, msg):
#         if len(msg.data) < 2:
#             self.get_logger().error("Message must contain at least [theta1, theta2]")
#             return

#         theta1 = float(msg.data[0])
#         theta2 = float(msg.data[1])

#         # Build transformation matrices
#         T01 = np.array([[math.cos(theta1), -math.sin(theta1), 0, 0],
#                         [math.sin(theta1),  math.cos(theta1), 0, 0],
#                         [0,                 0,                1, 0.133],
#                         [0,                 0,                0, 1]])

#         T12 = np.array([[math.cos(theta2), -math.sin(theta2), 0, 0.5],
#                         [math.sin(theta2),  math.cos(theta2), 0, 0],
#                         [0,                 0,                1, 0],
#                         [0,                 0,                0, 1]])

#         T23 = np.array([[1, 0, 0, 0.5],
#                         [0, 1, 0, 0],
#                         [0, 0, 1, 0],
#                         [0, 0, 0, 1]])

#         # Forward kinematics
#         T03 = T01 @ T12 @ T23

#         # Extract x, y
#         x = T03[0, 3]
#         y = T03[1, 3]

#         # Publish result
#         out_msg = Float32MultiArray()
#         out_msg.data = [x, y]
#         self.publisher.publish(out_msg)

#         self.get_logger().info(f"Angles: θ1={theta1:.3f}, θ2={theta2:.3f} -> End-effector: x={x:.3f}, y={y:.3f}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ForwardKinematicsNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
import math

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # Subscribe to the same topic that carries node IDs + angles
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/odrive/angle_cmd',
            self.listener_callback,
            10)

        # Publisher for end-effector coordinates
        self.publisher = self.create_publisher(Float32MultiArray, '/end_effector_xy', 10)

    def listener_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().error("Message must contain [node1_id, angle1, node2_id, angle2]")
            return

        # Extract angles correctly
        theta1 = float(msg.data[1])  # angle for joint 1
        theta2 = float(msg.data[3])  # angle for joint 2

        # Link lengths (from your DH setup)
        L1 = 0.15
        L2 = 0.15

        # Forward kinematics for planar 2-link SCARA
        x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

        # Publish result
        out_msg = Float32MultiArray()
        out_msg.data = [x, y]
        self.publisher.publish(out_msg)

        self.get_logger().info(
            f"Angles: θ1={theta1:.3f}, θ2={theta2:.3f} -> End-effector: x={x:.3f}, y={y:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
