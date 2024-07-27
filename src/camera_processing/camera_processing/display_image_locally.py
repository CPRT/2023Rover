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
                ('depth_history', 10),
                ('encoding', 'passthrough'),
                ('is_depth_image', False)
            ]
        )

        self.is_image_compressed = bool(self.get_parameter('is_image_compressed').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.image_encoding = str(self.get_parameter('encoding').value)
        self.is_depth_image = bool(self.get_parameter('is_depth_image').value)

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
        self.get_logger().info(f"Window name: {self.window_name}")
        self.get_logger().info(f"Encoding: {self.image_encoding}")
        self.get_logger().info(f"Depth image: {self.is_depth_image}")

    def image_callback(self, image):
        if self.is_image_compressed:
            img = self.cv_bridge.compressed_imgmsg_to_cv2(image, self.image_encoding)
        else:
            img = self.cv_bridge.imgmsg_to_cv2(image, self.image_encoding)

        if self.is_depth_image:
            img = cv2.equalizeHist(img)

        cv2.imshow(self.window_name, img)
                   
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