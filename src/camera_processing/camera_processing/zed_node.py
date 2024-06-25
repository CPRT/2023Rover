#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import pyzed.sl as sl

from typing import List
from threading import Lock, Thread
from time import sleep

# import ogl_viewer.viewer as gl
# import cv_viewer.tracking_viewer as cv_viewer

from camera_processing.camera_processing.zed_helper_files.detect_vision_targets import DetectVisionTargets, CameraType
from camera_processing.camera_processing.zed_helper_files.video_capture import VideoCapture

class ZedNode(rclpy.Node):
    def __init__(self):
        self.detectVisionTargets = DetectVisionTargets()
        self.ir_cam = VideoCapture(0, CameraType.ERIK_ELP)

        self.init_zed(self._zed)

        self.image_left_tmp = sl.Mat()
        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.runtime_params = sl.RuntimeParameters()

        self.timer_period = 0.05  # 0.066 for 15 FPS
        self.timer = self.create_timer(self.timer_period, self.run_detections)

        self.timestamp = self.get_clock().now()

    def init_zed(self):
        self.timestamp = self.get_clock().now()
        self.get_logger().info("Initializing ZED Camera")

        self.zed = sl.Camera()

        input_type = sl.InputType()
        # if opt and opt.svo is not None:
        #     input_type.set_from_svo_file(opt.svo)

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
        init_params.camera_resolution = sl.RESOLUTION.HD2K   # HD1080   HD1200    HD2K
        init_params.camera_fps = 15 # Use 15 FPS to improve low-light performance
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Can use PERFORMANCE MODE but it misses some details
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = 20

        status = self.zed.open(init_params)

        if status != sl.ERROR_CODE.SUCCESS:
            print("Failed to init ZED")
            print(repr(status))
            exit()
        
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        self.zed.enable_object_detection(obj_param)

        # Get Camera resolution
        camera_infos = self.zed.get_camera_information()
        camera_res = camera_infos.camera_configuration.resolution

        self.get_logger().info(f"Finished initializing ZED Camera in {self.get_clock().now() - self.timestamp} seconds")

    def cleanup(self):
        self.zed.close()
        cv2.destroyAllWindows()

    def run_detections(self):
        self.timestamp = self.get_clock().now()
        self.get_logger().info(f"Timer callback - Running zed.grab blocking call")

        # grab is a blocking call
        error_code = self.zed.grab(self.runtime_params)
        if error_code != sl.ERROR_CODE.SUCCESS:
            # TODO: Add handling for a camera error. Most likely error here is ERROR_CODE.CAMERA_NOT_DETECTED
            return
        
        self.get_logger().info(f"zed.grab finished after {self.get_clock().now() - self.timestamp} seconds")
        self.timestamp = self.get_clock().now()

        detections: List[sl.CustomBoxObjectData] = []

        # ZED Image
        self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
        zed_left_image_net = self.image_left_tmp.get_data()
        zed_img = cv2.cvtColor(zed_left_image_net, cv2.COLOR_BGRA2RGB)

        # ZED Aruco Markers
        # detections += detectVisionTargets.detectArucoMarkers(zed_img, CameraType.ZED)

        # ZED Red/Blue LEDS
        # detections += detectVisionTargets.detectZEDLEDs()

        # IR Cam Image
        ir_image = self.ir_cam.read()

        # IR Cam Aruco Markers
        detections += self.detectVisionTargets.detectArucoMarkers(ir_image, self.ir_cam.cam_type)

        # IR Cam LEDs
        # detections += detectVisionTargets.detectIRLEDS(ir_image, ir_cam.cam_type)

        # Ingest detections and get objects
        self.zed.ingest_custom_box_objects(detections)
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        # Print object detections
        for object in self.objects.object_list:
            print(f"Object ~ Id: {object.id()}, Unique Label: {object.unique_object_id()}, position: {object.position()}")
            self.detectVisionTargets.draw_object_detection(zed_img, object)

        self.get_logger().info(f"Zed computations finished in {self.get_clock().now() - self.timestamp} seconds")

        cv2.imshow("ZED Detections", zed_img)

def main(args=None):
    rclpy.init(args=args)

    node = ZedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
