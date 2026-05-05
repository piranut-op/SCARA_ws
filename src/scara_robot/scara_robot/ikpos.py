# import rclpy
# from rclpy.node import Node
# import math
# from std_msgs.msg import Float32MultiArray

# class InverseKinematicsNode(Node):
#     def __init__(self):
#         super().__init__('inverse_kinematics_node')

#         # Publisher: send angles to /odrive/angle_cmd
#         self.publisher_ = self.create_publisher(Float32MultiArray, '/odrive/angle_cmd', 10)

#         # Subscriber: listen for target (x, y) on /ik_target
#         self.subscription = self.create_subscription(
#             Float32MultiArray,
#             '/ik_target',
#             self.listener_callback,
#             10
#         )

#     def listener_callback(self, msg):
#         # Expect msg.data = [x, y]
#         if len(msg.data) < 2:
#             self.get_logger().error("Received invalid IK target, need [x, y]")
#             return

#         x_target = msg.data[0]
#         y_target = msg.data[1]

#         # Link lengths
#         a1 = 0.15
#         a2 = 0.15

#         # Step 1: compute r^2
#         r2 = x_target**2 + y_target**2

#         # Step 2: compute theta2
#         cos_theta2 = (r2 - a1**2 - a2**2) / (2 * a1 * a2)
#         cos_theta2 = max(min(cos_theta2, 1.0), -1.0)  # clamp
#         theta2 = math.acos(cos_theta2)

#         # Step 3: compute theta1
#         theta1 = math.atan2(y_target, x_target) - math.atan2(a2 * math.sin(theta2), a1 + a2 * math.cos(theta2))

#         # Prepare message with 4 values (example format)
#         out_msg = Float32MultiArray()
#         out_msg.data = [1.0, theta1, 2.0, theta2]  # match CLI style

#         # Publish angles
#         self.publisher_.publish(out_msg)
#         self.get_logger().info(f'IK target=({x_target:.2f},{y_target:.2f}) -> published {out_msg.data}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = InverseKinematicsNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()








import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32MultiArray

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Publisher: send angles to /odrive/angle_cmd
        self.publisher_ = self.create_publisher(Float32MultiArray, '/odrive/angle_cmd', 10)

        # Subscriber: listen for target (x, y) on /ik_target
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ik_target',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Expect msg.data = [x, y]
        if len(msg.data) < 2:
            self.get_logger().error("Received invalid IK target, need [x, y]")
            return

        x_target = msg.data[0]
        y_target = msg.data[1]

        # Workspace limits (inclusive)
        x_min, x_max = -0.159, 0.159
        y_min, y_max = 0.1, 0.39

        # Reject if outside workspace (inclusive check)
        if x_target < x_min or x_target > x_max or y_target < y_min or y_target > y_max:
            self.get_logger().warn(
                f"Target ({x_target:.2f},{y_target:.2f}) outside workspace "
                f"x:[{x_min},{x_max}], y:[{y_min},{y_max}]"
            )
            return
        else:
            self.get_logger().info(
                f"Target ({x_target:.2f},{y_target:.2f}) inside workspace"
            )

        # Link lengths
        a1 = 0.15
        a2 = 0.15

        # Step 1: compute r^2
        r2 = x_target**2 + y_target**2

        # Step 2: compute theta2
        cos_theta2 = (r2 - a1**2 - a2**2) / (2 * a1 * a2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)  # clamp
        theta2 = math.acos(cos_theta2)

        # Step 3: compute theta1
        theta1 = math.atan2(y_target, x_target) - math.atan2(
            a2 * math.sin(theta2), a1 + a2 * math.cos(theta2)
        )

        # Prepare message with 4 values (example format)
        out_msg = Float32MultiArray()
        out_msg.data = [1.0, theta1, 2.0, theta2]  # match CLI style

        # Publish angles
        self.publisher_.publish(out_msg)
        self.get_logger().info(
            f'IK target=({x_target:.2f},{y_target:.2f}) -> published {out_msg.data}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
