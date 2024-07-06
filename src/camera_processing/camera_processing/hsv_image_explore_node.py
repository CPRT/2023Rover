from camera_processing.camera_processing.HSVImageExplore.image_colour_processing.colour_processing import ColourProcessing
import rclpy, cv2
from rclpy.node import Node 
from rclpy import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class TuneHSVImageExplore(Node):
    def __init__(self):
        super().__init__('display_image_locally')

        self.declare_parameters(
            namespace="",
            parameters=[
                ('window_name', 'DisplayImage'),
                ('image_topic', '/zed/zed_raw_image'),
                ('is_image_compressed', False),
                ('depth_history', 10),

                # options are blue, red, or ir
                ('pipeline', 'blue'),

                # From colour_processing_params.yaml. This yaml file is required
                ('resize_for_processing', Parameter.Type.DOUBLE),
                ('blue_led', Parameter.Type.STRING),
                ('red_led', Parameter.Type.STRING),
                ('ir_led', Parameter.Type.STRING),
            ]
        )

        if 'blue' in self.get_parameter('pipeline').value:
            colour_processing_str = str(self.get_parameter('blue_led').value)
        elif 'red' in self.get_parameter('pipeline').value:
            colour_processing_str = str(self.get_parameter('red_led').value)
        elif 'ir' in self.get_parameter('pipeline').value:
            colour_processing_str = str(self.get_parameter('ir_led').value)
        
        self.colour_processing = ColourProcessing.from_string(colour_processing_str)
        if not isinstance(self.colour_processing, ColourProcessing):
            raise ValueError(f"Invalid colour processing string: {colour_processing_str}")

        self.image = None
        self.image_count = 0
        self.image_proceessed_count = -1

        self.is_image_compressed = bool(self.get_parameter('is_image_compressed').value)

        self.image_subscriber = self.create_subscription(
            CompressedImage if self.is_image_compressed else Image,
            str(self.get_paramter('image_topic').value),
            self.image_callback, 
            int(self.get_paramter('depth_history').value)
        )

        self.cv_bridge = CvBridge()

    def image_callback(self, image):
        self.image_count += 1
        if self.is_image_compressed:
            self.image = self.cv_bridge.compressed_imgmsg_to_cv2(image)
        else:
            self.image = self.window_name, self.cv_bridge.imgmsg_to_cv2(image)
            
    def run_hsv_image_explore(self):
        if self.image is not None and self.image_count != self.image_proceessed_count:
            self.image_proceessed_count = self.image_count
            self.colour_processing.mask_step_tuning(self.image)

def main(args=None):
    rclpy.init(args=args)

    node = TuneHSVImageExplore()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()