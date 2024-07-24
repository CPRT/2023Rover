import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

import os, re, glob
from threading import Thread
from typing import List
from pathlib import Path

from .video_capture import VideoCapture

class CameraDetails:
    def __init__(self, xRes: int, yRes: int, name: str, v4l_byid_name: str,):
        self.v4l_byid_name = v4l_byid_name
        self.xRes = xRes
        self.yRes = yRes
        self.name = name

class ScienceCaptureAll(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Empty, 'science_capture_all', self.science_capture_all)

        self.science_directory = os.path.join(Path.home(), "Documents", "ImagesForScience")
        self.num_images = 10
        self.time_between_images = 0.2
        self.current_directory = os.path.join(self.science_directory, "error")

        self.image_count = 0

        # Look at names in /dev/v4l/by-id/
        self.camera_details = [
            CameraDetails(640, 480, "front_camera", "usb-CN0HK46K8LG00114H364A03_Integrated_Webcam_HD_200901010001-video-index0"),
        ]

        self.get_logger().info("Directory: " + str(self.science_directory))

    def setup_images_directory(self) -> str:
        self.image_count += 1
        dir_name = f"Test{self.image_count}"
        os.makedirs(os.path.join(self.science_directory, dir_name), exist_ok=True) 
        return os.path.join(self.science_directory, dir_name)

    def science_capture_all(self, request: Empty, response: Empty):
        try:
            self.current_directory = self.setup_images_directory()
            self.get_logger().info("Saving images to " + str(self.current_directory))

            v4l_byid_names = self._get_all_video_names()
            
            threads: List[Thread] = []
            for name in v4l_byid_names:
                for camera in self.camera_details:
                    if camera.v4l_byid_name in name:
                        t = Thread(target=self.capture_images, args=[camera])
                        t.start()
                        threads.append(t)

            for t in threads:
                t.join()
        except Exception as e:
            self.get_logger().error("Failed to capture images with error " + str(e))
        finally:
            return response
    
    def capture_images(self, camera_details: CameraDetails):
        try:
            index: int = self._find_video_index(camera_details.v4l_byid_name)
        except ValueError as e:
            self.get_logger().error(f"Error when finding the video index for the camera. Error: {e}")
            return

        vid_capture = VideoCapture(
            cam_index=index,
            xRes=camera_details.xRes,
            yRes=camera_details.yRes,
            filename_prefix=camera_details.name,
            output_directory=self.current_directory
        )

        vid_capture.save_images(self.num_images, self.time_between_images)

        self.get_logger().info(f"Saved {self.num_images} images from {camera_details.name} to {vid_capture.output_directory}")
        vid_capture.close()

    def _get_all_video_names(self):
        v4l_byid_dir = "/dev/v4l/by-id/"
        if not os.path.exists(v4l_byid_dir):
            raise ValueError("Could not find the directory /dev/v4l/by-id/")
        
        return glob.glob(v4l_byid_dir + "*index0")

    def _find_video_index(self, v4l_byid_name: str):
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