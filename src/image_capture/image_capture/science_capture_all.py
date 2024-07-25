import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

import os, re, glob, subprocess
from copy import deepcopy
from threading import Thread
from typing import List
from pathlib import Path

from .video_capture import VideoCapture

class CameraDetails:
    def __init__(self, xRes: int, yRes: int, name: str, serial_name: str, v4l_byid_name: str,):
        self.v4l_byid_name = v4l_byid_name
        self.serial_name = serial_name
        self.xRes = xRes
        self.yRes = yRes
        self.name = name
        self.device_path = "error"

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
            CameraDetails(640, 480, "front_camera", "CN0HK46K8LG00114H364A03_Integrated_Webcam_HD_200901010001", ""),
            CameraDetails(1920, 1080, "elp", "HD_Camera_Manufacturer_USB_2.0_Camera", ""),
            CameraDetails(1920, 1080, "lowlight", "e-con_systems_See3CAM_CU27_3B1519112B010900", "")
            # CameraDetails(1920, 1080, "elp2", "", "usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0")
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

            cam_details = self.get_all_cameras()

            threads: List[Thread] = []
            for camera in cam_details:
                # if self._check_device_name(camera.v4l_byid_name, camera.device_name):
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
            vid_capture = VideoCapture(
                cam_index=self._get_index_from_device_path(camera_details.device_path),
                xRes=camera_details.xRes,
                yRes=camera_details.yRes,
                filename_prefix=camera_details.name,
                output_directory=self.current_directory
            )

            vid_capture.save_images(self.num_images, self.time_between_images)

            self.get_logger().info(f"Saved {self.num_images} images from {camera_details.name} to {vid_capture.output_directory}")
            vid_capture.close()
        except Exception as e:
            self.get_logger().info(f"Failed to capture images from {str(camera_details.name)} with error: {e}")

    def _get_all_video_names(self):
        v4l_byid_dir = "/dev/v4l/by-id/"
        if not os.path.exists(v4l_byid_dir):
            raise ValueError("Could not find the directory /dev/v4l/by-id/")
        
        return glob.glob(v4l_byid_dir + "*index0")

    def get_all_cameras(self) -> List[CameraDetails]:
        cam_details: List[CameraDetails] = []
        video_devices = glob.glob("/dev/video*")

        self.get_logger().info("Video devices: " + repr(video_devices))

        for device_path in video_devices:
            if not self._is_video_capture_index(device_path):
                self.get_logger().info(f"Rejecting {device_path} because it is not a capturing device")
                continue

            serial = self._get_cam_serial(device_path)
            self.get_logger().info("Serial: " + str(serial))

            for camera in self.camera_details:
                if camera.serial_name == serial:
                    if camera in cam_details:
                        copy_camera = deepcopy(camera)
                        copy_camera.device_path = device_path
                        copy_camera.name += "2"
                        cam_details.append(copy_camera)

                    else:

                        camera.device_path = device_path
                        cam_details.append(camera)

                    break
        
        self.get_logger().info("Number of cameras found " + str(len(cam_details)))
        
        for details in cam_details:
            self.get_logger().info(f"Name: {details.name}, Device: {details.device_path}")

        cam_details = [x for x in cam_details if x.device_path != "error"]

        return cam_details
    
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
        
        self.get_logger().info(f"Camera info {info}")

        return int(info.group(1))

    def _get_cam_serial(self, cam_dev_path: str):
        # Prepare the external command to extract serial number. 
        p = subprocess.Popen('udevadm info --name={} | grep ID_SERIAL= | cut -d "=" -f 2'.format(cam_dev_path),
                            stdout=subprocess.PIPE, shell=True)

        # Command is: udevadm info --name=/dev/video0 | grep ID_SERIAL= | cut -d "=" -f 2

        # Run the command
        (output, err) = p.communicate()

        # Wait for it to finish
        p.status = p.wait()

        # Decode the output
        response = output.decode('utf-8')

        # The response ends with a new line so remove it
        return response.replace('\n', '')

    def _is_video_capture_index(self, cam_dev_path: str):
        p = subprocess.Popen(f'v4l2-ctl --list-formats-ext -d {cam_dev_path}', stdout=subprocess.PIPE, shell=True)

        # Run the command
        (output, err) = p.communicate()

        # Wait for it to finish
        p.status = p.wait()

        # Decode the output
        response = output.decode('utf-8')

        self.get_logger().info("Video capture formats for " + cam_dev_path + ": " + repr(response))

        if "MJPG" in response:
            return True
        
        if "YUYV" in response:
            return True
        
        if "UYVY" in response:
            return True

        return False
    
    def _get_index_from_device_path(self, device_path: str) -> int:
        device_re = re.compile("\/dev\/video(\d+)")
        info = device_re.match(device_path)
        if not info:
            raise ValueError("Could not find the video index in the file pointed to by v4l_byid_name")
        
        self.get_logger().info(f"Camera info {info}")

        return int(info.group(1))


def main():
    rclpy.init()
    node = ScienceCaptureAll()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()