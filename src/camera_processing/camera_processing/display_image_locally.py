import rclpy, cv2
from rclpy.node import Node 
from rclpy import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class DisplayImageLocally(Node):
    def __init__(self):
        super().__init__('display_image_locally')

        self.declare_parameters(
            namespace="",
            parameters=[
                ('window_name', 'DisplayImage'),
                ('image_topic', '/cv_zed_image'),
                ('is_image_compressed', True),
                ('depth_history', 10)
            ]
        )

        self.is_image_compressed = bool(self.get_parameter('is_image_compressed').value)
        self.window_name = str(self.get_parameter('window_name').value)

        self.image_subscriber = self.create_subscription(
            CompressedImage if self.is_image_compressed else Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback, 
            int(self.get_parameter('depth_history').value)
        )

        self.cv_bridge = CvBridge()
        self.image_count = 0

        self.get_logger().info(f"Subscribed to {self.get_parameter('image_topic').value}")
        self.get_logger().info(f"Expecting {'CompressedImage' if self.is_image_compressed else 'Image'} message")

    def image_callback(self, image):
        if self.is_image_compressed:
            cv2.imshow(self.window_name, self.cv_bridge.compressed_imgmsg_to_cv2(image))
        else:
            cv2.imshow(self.window_name, self.cv_bridge.imgmsg_to_cv2(image))

        self.image_count += 1
        self.get_logger().info(f"Image count: {self.image_count}")
        cv2.waitKey(500)


def main(args=None):
    rclpy.init(args=args)

    node = DisplayImageLocally()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()