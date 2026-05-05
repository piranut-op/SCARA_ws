import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',   # Webcam feed topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Use absolute path to Haar cascade file
        self.face_cascade = cv2.CascadeClassifier(
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        )

    def listener_callback(self, msg):
        # Convert ROS2 Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            # Draw rectangle around face
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            self.get_logger().info(f"Face detected at: x={x}, y={y}, w={w}, h={h}")

        # Show video with detection overlay
        cv2.imshow("Face Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
