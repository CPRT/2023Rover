import rclpy
import rclpy.logging
import random
from rclpy.node import Node
from interfaces.srv import VideoOut
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


class WebRTCStreamer(Node):
    def __init__(self):
        Gst.init(None)
        super().__init__("WebRTCStreamerNode")
        self.declare_parameter("source_list", {})
        self.source_list = (
            self.get_parameter("source_list").get_parameter_value().string_dict_value
        )
        self.declare_parameter("web_server", False)
        self.web_server = (
            self.get_parameter("web_server").get_parameter_value().bool
        )
        self.start = self.create_service(VideoOut, 'start_video', self.start_video_cb)

    def start_video_cb(self, request, response):
        pipeline_str = self.create_pipeline(request)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.get_logger().info(pipeline_str)
        self.pipeline.set_state(Gst.State.PLAYING)
        response = True
        return response


    def create_pipeline(self, request):
        pipeline = ""
        compositor = "nvcompositor name=mix"
        total_width = request.width
        total_height = request.height
        i = 0
        for input in request.sources:
            name = input.name
            height = int(input.height * total_height / 100)
            width = int(input.width * total_width / 100)
            origin_x = int(input.origin_x * total_width / 100)
            origin_y = int(input.origin_y * total_height / 100)
            pipeline += f"v4l2src device={self.source_list[name]} ! nvvidconv ! mix.sink_{i} "
            compositor += f" sink_{i}::xpos={origin_x} sink_{i}::ypos={origin_y} sink_{i}::height={height} sink_{i}::width={width}"
            i = i+1
        videoOut = "webrtcsink run-signalling-server=true"
        if self.web_server:
            videoOut += " run-web-server=true web-server-host-addr=http://0.0.0.0:8080/"
        pipeline += compositor + " ! " + videoOut
        return pipeline
    


        

def main(args=None):
    rclpy.init(args=args)
    node = jpegPoC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()