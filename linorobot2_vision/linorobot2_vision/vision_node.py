import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class WebcamNode(Node):
    def __init__(self):
        super().__init__('linorobot2_vision')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Modify the image (e.g., convert to grayscale)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Convert OpenCV image back to ROS Image message
            modified_image_msg = self.bridge.cv2_to_imgmsg(gray_image, "mono8")

            # Publish the modified image
            self.publisher_.publish(modified_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
