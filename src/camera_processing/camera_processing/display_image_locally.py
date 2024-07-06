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
                ('image_topic', '/zed/cv_zed_image'),
                ('is_image_compressed', True),
                ('depth_history', 10)
            ]
        )

        self.is_image_compressed = bool(self.get_parameter('is_image_compressed').value)

        self.image_subscriber = self.create_subscription(
            CompressedImage if self.is_image_compressed else Image,
            str(self.get_paramter('image_topic').value),
            self.image_callback, 
            int(self.get_paramter('depth_history').value)
        )

        self.cv_bridge = CvBridge()

    def image_callback(self, image):
        if self.is_image_compressed:
            cv2.imshow(self.window_name, self.cv_bridge.compressed_imgmsg_to_cv2(image))
        else:
            cv2.imshow(self.window_name, self.cv_bridge.imgmsg_to_cv2(image))

        cv2.waitKey(5)


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