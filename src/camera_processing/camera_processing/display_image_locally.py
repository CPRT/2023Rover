import rclpy, cv2
from rclpy.node import Node 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class DisplayImageLocally(Node):
    def __init__(self):
        super().__init__('display_image_locally')

        self.name = "DisplayImage"
        image_topic = '/acruco_processed'
        self.is_image_compressed = True
        depth_history = 10

        self.image_subscriber = self.create_subscription(
            CompressedImage if self.is_image_compressed else Image,
            image_topic,
            self.image_callback, 
            depth_history
        )

        self.br = CvBridge()

    def image_callback(self, image):
        if self.is_image_compressed:
            cv2.imshow(self.name, self.br.compressed_imgmsg_to_cv2(image))
        else:
            cv2.imshow(self.name, self.br.imgmsg_to_cv2(image))

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