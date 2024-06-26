#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import pyzed.sl as sl

from rclpy.node import Node
# import rclpy.time as Time
from typing import List
from threading import Lock, Thread
from time import sleep

from geometry_msgs.msg import Point
from cprt_interfaces.msg import PointArray, ArucoMarkers

# import ogl_viewer.viewer as gl
# import cv_viewer.tracking_viewer as cv_viewer

from .zed_helper_files.detect_vision_targets import DetectVisionTargets, CameraType
from .zed_helper_files.video_capture import VideoCapture

class ZedNode(Node):
    def __init__(self):
        super().__init__('zed_node')
        self.frame_id = "/zed_link"
        self.record_svo = False
        self.playback_svo = False
        self.record_filename = "ZED_SVO"
        self.playback_filename = "ZED_SVO"

        self.detectVisionTargets = DetectVisionTargets()
        self.ir_cam = VideoCapture(0, CameraType.ERIK_ELP)
        self.init_zed()
   
        self.image_left_tmp = sl.Mat()
        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.runtime_params = sl.RuntimeParameters()

        self.timer_period = 0.05  # 0.066 for 15 FPS
        self.timer = self.create_timer(self.timer_period, self.run_detections)

        self.publish_zed_aruco_points = self.create_subscription(ArucoMarkers, '/vision/zed_aruco_points', 10)
        self.publish_blue_led_points = self.create_subscription(PointArray, '/vision/blue_led_points', 10)
        self.publish_red_led_points = self.create_subscription(PointArray, '/vision/red_led_points', 10)
        self.publish_ir_led_points = self.create_subscription(PointArray, '/vision/ir_led_points', 10)

        self.timestamp = self.get_clock().now()
        self.header_timestamp = self.get_clock().now().to_msg()

    def init_zed(self):
        self.timestamp = self.get_clock().now()
        self.get_logger().info("Initializing ZED Camera")

        self.zed = sl.Camera()

        input_type = sl.InputType()
        if self.playback_svo and self.playback_filename != "":
            input_type.set_from_svo_file(self.playback_filename)
            self.get_logger().info(f"Playing back SVO. Data from the ZED is a recording and NOT LIVE. Filename: {self.playback_filename}")

        elif self.record_svo and self.record_filename != "":
            recordingParameters = sl.RecordingParamers()
            recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
            recordingParameters.video_filename = self.record_filename
            err = self.zed.enable_recording(recordingParameters)
            
        

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
        init_params.camera_resolution = sl.RESOLUTION.HD1080   # HD1080   HD1200    HD2K
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
        self.zed.disable_recording()
        self.zed.close()
        cv2.destroyAllWindows()

    def run_detections(self):
        self.timestamp = self.get_clock().now()
        self.get_logger().info(f"Timer callback - Running zed.grab blocking call")

        # grab is a blocking call
        error_code = self.zed.grab(self.runtime_params)
        if error_code != sl.ERROR_CODE.SUCCESS:
            # TODO: Add handling for a camera error. Most likely error here is ERROR_CODE.CAMERA_NOT_DETECTED

            if error_code == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                self.get_logger().info("SVO end has been reached. Looping back to first frame")
                self.zed.set_svo_position(0)

            return
        
        self.header_timestamp = self.get_clock().now().to_msg()

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
        # ir_image = self.ir_cam.read()

        # IR Cam Aruco Markers
        # detections += self.detectVisionTargets.detectArucoMarkers(ir_image, self.ir_cam.cam_type)

        # IR Cam LEDs
        # detections += detectVisionTargets.detectIRLEDS(ir_image, ir_cam.cam_type)

        # Ingest detections and get objects
        self.zed.ingest_custom_box_objects(detections)
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        # Iterate object detections
        zed_aruco_markers_msg = self.create_aruco_markers_msg()
        blue_led_point_arr = self.create_point_array()
        red_led_point_arr = self.create_point_array()
        ir_led_point_arr = self.create_point_array()

        for object in self.objects.object_list:
            print(f"Object ~ Id: {object.id()}, Unique Label: {object.unique_object_id()}, position: {object.position()}")
            self.detectVisionTargets.draw_object_detection(zed_img, object)
            point: Point = self.zed_object_to_point(object)

            if (object.unique_object_id() == "zed_aruco"):
                zed_aruco_markers_msg.points.append(point)
                zed_aruco_markers_msg.marker_ids.append(object.get_label())

            elif (object.unique_object_id() == "blue_led"):
                blue_led_point_arr.points.append(point)

            elif (object.unique_object_id() == "red_led"):
                red_led_point_arr.points.append(point)

            elif (object.unique_object_id() == "ir_led"):
                ir_led_point_arr.points.append(point)
        
        self.publish_zed_aruco_points(zed_aruco_markers_msg)
        self.publish_blue_led_points(blue_led_point_arr)
        self.publish_red_led_points(red_led_point_arr)
        self.publish_ir_led_points(ir_led_point_arr)

        self.get_logger().info(f"Zed computations finished in {self.get_clock().now() - self.timestamp} seconds")

        cv2.imshow("ZED Detections", zed_img)
        cv2.waitKey(10)

    def create_aruco_markers_msg(self) -> ArucoMarkers:
        markers = ArucoMarkers()
        markers.header.frame_id = self.frame_id
        markers.stamp = self.header_timestamp
        return markers

    def create_point_array(self) -> PointArray:
        point_arr = PointArray()
        point_arr.header.frame_id = self.frame_id
        point_arr.stamp = self.header_timestamp
        return point_arr

    def zed_object_to_point(self, object: sl.ObjectData) -> Point:
        point = Point()
        point.x = object.position[0]
        point.y = object.position[1]
        point.z = object.position[2]

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