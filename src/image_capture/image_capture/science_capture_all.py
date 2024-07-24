import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

import os, re

class ScienceCaptureAll(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Empty, 'science_capture_all', self.science_capture_all)

    def science_capture_all(self, request: Empty, response: Empty):



        return response
    
    @classmethod
    def _find_video_index(v4l_byid_name: str):
        if v4l_byid_name is None or len(v4l_byid_name) == 0:
            raise ValueError("Must provide a v4l_byid_name name to find the video index")
        
        v4l_byid_name = "/dev/v4l/by-id/" + v4l_byid_name

        if not os.path.exists(v4l_byid_name):
            raise ValueError("Path doesn't exist when following the v4l_byid_name")
        
        device_path = os.path.realpath(v4l_byid_name)
        device_re = re.compile("\/dev\/video(\d+)")
        info = device_re.match(device_path)
        if not info:
            raise ValueError("Could not find the video index in the file pointed to by v4l_byid_name")
        
        return int(info.group(1))

def main():
    rclpy.init()
    node = ScienceCaptureAll()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()