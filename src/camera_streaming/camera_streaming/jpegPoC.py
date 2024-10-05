import rclpy
import rclpy.logging
import random
from rclpy.node import Node
from interfaces.srv import ChangeRes
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

pipeline_name = 'p0'


class jpegPoC(Node):
    def __init__(self):
        Gst.init(None)
        super().__init__("JpegPoCNode")
        self.declare_parameter("port", 7001)
        self.target_host = (
            self.get_parameter("port").get_parameter_value().integer_value
        )
        self.declare_parameter("host", "localhost")
        self.target_host = (
            self.get_parameter("host").get_parameter_value().string_value
        )
        self.declare_parameter("device", "/dev/video0")
        self.device = (
            self.get_parameter("device").get_parameter_value().string_value
        )
        self.declare_parameter("height", 1080)
        self.height = (
            self.get_parameter("height").get_parameter_value().integer_value
        )
        self.declare_parameter("width", 1920)
        self.width = (
            self.get_parameter("width").get_parameter_value().integer_value
        )
        self.declare_parameter("interval", 1000)
        self.interval = (
            self.get_parameter("interval").get_parameter_value().integer_value
        )
        self.timer = self.create_timer(self.interval/1000, self.check_bandwidth)
        self.srv = self.create_service(ChangeRes, 'change_resolution', self.change_res_callback)
        self.frame_rate = -1
        self.create_pipeline()


    def create_pipeline(self):
        self.pipeline = Gst.parse_launch(f'v4l2src device={self.device} do-timestamp=true ! videorate name=rate ! capsfilter name=caps ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000')
        self.pipeline.set_state(Gst.State.PLAYING)
    
    def change_res_callback(self, request, response):
        response.success = True
        return response
    
    def check_bandwidth(self):
        # To be implemented(get bandwidth estimate from link)
        # For now, pick a random number between 1 and 100
        bandwidth = random.randint(10, 100)
        in_use = random.randint(1, bandwidth)
        # --------------------- #
        usage = in_use / bandwidth
        self.update_bandwidth(usage)
        self.get_logger().info(f"Bandwidth: {bandwidth} in use: {in_use} ({usage * 100:.2f}%), frame rate: {self.frame_rate}")
        return
    
    def update_bandwidth(self, usage):
        if usage > 0.9:
            self.frame_rate = int(self.frame_rate * 0.5)
        elif usage > 0.8:
            self.frame_rate = int(self.frame_rate * 0.8)
        elif usage < 0.4:
            self.frame_rate = int(self.frame_rate * 1.5) + 1
        elif usage < 0.5:
            self.frame_rate = int(self.frame_rate * 1.1) + 1
        else:
            return
        if self.frame_rate < 1:
            self.frame_rate = 1
        elif self.frame_rate > 60:
            self.frame_rate = 60
        self.setRate()

    def refreshCaps(self):
        element = self.pipeline.get_by_name("caps")
        if not element:
            self.get_logger().error("Element not found")
            return
        self.pipeline.set_state(Gst.State.READY)
        new_caps = Gst.Caps.from_string(f"image/jpeg, height={self.height}, width={self.width}")
        element.set_property("caps", new_caps)
        self.pipeline.set_state(Gst.State.PLAYING)

    def setRate(self):
        element = self.pipeline.get_by_name("rate")
        if not element:
            self.get_logger().error("Element not found")
            return
        self.pipeline.set_state(Gst.State.READY)
        element.set_property("max-rate", self.frame_rate)
        self.pipeline.set_state(Gst.State.PLAYING)

def main(args=None):
    rclpy.init(args=args)
    node = jpegPoC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()