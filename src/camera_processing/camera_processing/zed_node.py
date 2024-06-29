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

from .zed_viewers.cv_viewer import tracking_viewer as cv_viewer
from .zed_viewers.ogl_viewer import viewer as gl

from .zed_helper_files.detect_vision_targets import DetectVisionTargets, CameraType
from .zed_helper_files.video_capture import VideoCapture
from .HSVImageExplore.image_colour_processing.colour_processing import ColourProcessing
from .HSVImageExplore.image_colour_processing.hsv_range_mask_step import HSVRangeMaskStep
from .HSVImageExplore.image_colour_processing.simple_mask_steps import ErodeDilateStep
from .HSVImageExplore.image_colour_processing.datatypes import HSVRange, HSV

class ZedNode(Node):
    def __init__(self):
        super().__init__('zed_node')
        self.frame_id = "/zed_link"
        self.record_svo = False
        self.playback_svo = True
        self.hsv_explore = True
        self.record_filename = "/home/jetson/Documents/ZED/Jun28/recording/test"
        # self.playback_filename = "/home/jetson/Documents/ZED/Jun27/HD1080_SN32985835_23-09-27-ArucoMarkers.svo2"
        # self.playback_filename = "/home/jetson/Documents/ZED/Jun27/HD1080_SN32985835_23-08-25-RedBlueBalls.svo2"
        # self.playback_filename = "/home/jetson/Documents/ZED/Jun28/HD1080_SN32985835_20-57-10-BlueLED-Darker.svo2"

        # self.playback_filename = "/home/jetson/Documents/ZED/Jun28/" + "HD1080_SN32985835_21-47-45-Jun28-BlueLED-1MeterCalibrationGrid-ReflectiveCouch.svo2"
        # self.playback_filename = "/home/jetson/Documents/ZED/Jun28/" + "HD1080_SN32985835_21-43-38-Jun28-BlueLEDs-DeckLightOff.svo2"

        self.playback_filename = "/home/jetson/Documents/ZED/Jun28/temp/output.svo2"

        self.detectVisionTargets = DetectVisionTargets()
        # self.ir_cam = VideoCapture(0, CameraType.ERIK_ELP)

        zed_initialized = False
        while not zed_initialized:
            try:
                self.init_zed()
                zed_initialized = True
            except Exception as e:
                self.get_logger().error(f"Failed to initialize ZED. Exception: {e}")
                sl.Camera.reboot(sn=0, full_reboot=True) # Reboot fixes USB problems on Jetson Nano
    
        self.init_visualizers()

        self.image_left_tmp = sl.Mat()
        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.runtime_params = sl.RuntimeParameters()

        self.publish_zed_aruco_points = self.create_publisher(ArucoMarkers, '/vision/zed_aruco_points', 10)
        self.publish_blue_led_points = self.create_publisher(PointArray, '/vision/blue_led_points', 10)
        self.publish_red_led_points = self.create_publisher(PointArray, '/vision/red_led_points', 10)
        self.publish_ir_led_points = self.create_publisher(PointArray, '/vision/ir_led_points', 10)

        self.blue_led_processing = ColourProcessing(
            image_scaling=1.0, 
            display_scaling=0.3,
            mask_steps=tuple([
                HSVRangeMaskStep('Step 1 - HSV - White Filament', HSVRange(HSV(0, 0, 245), HSV(180, 50, 255)), return_mask=True), 
                ErodeDilateStep('Step 2 - Dilate to widen mask', erosion=-1, dilation=18, return_mask=False),
                HSVRangeMaskStep('Step 3 - HSV - Blue around white filament', HSVRange(HSV(103, 0, 109), HSV(130, 255, 253)), return_mask=True), 
                ErodeDilateStep('Step 4 - Dilate - Prevent disjointed contours', erosion=-1, dilation=3, return_mask=True),
            ]))

        self.timer_period = 0.05  # 0.066 for 15 FPS
        self.timer = self.create_timer(self.timer_period, self.run_detections)

        self.timestamp = self.get_clock().now()
        self.header_timestamp = self.get_clock().now().to_msg()

    def init_zed(self):
        self.record_timestamp()
        self.get_logger().info("Initializing ZED Camera")


        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters(svo_real_time_mode=False)
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Can use PERFORMANCE MODE but it misses some details (sl.DEPTH_MODE.ULTRA) (sl.DEPTH_MODE.PERFORMANCE)
        init_params.depth_maximum_distance = 15
        init_params.camera_resolution = sl.RESOLUTION.HD1080   # HD720   HD1080   HD1200    HD2K
        init_params.camera_fps = 15 # Use 15 FPS to improve low-light performance

        if self.playback_svo and self.playback_filename != "":
            init_params.set_from_svo_file(self.playback_filename)
            self.get_logger().info(f"Playing back SVO. Data from the ZED is a recording and NOT LIVE. Filename: {self.playback_filename}")
        
        self.zed = sl.Camera()
        status = self.zed.open(init_params)

        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception("Failed to zed.open(init_params). Got ZED error code: " + repr(status))
        
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS  # CUSTOM_BOX_OBJECTS  MULTI_CLASS_BOX_MEDIUM
        obj_param.enable_tracking = False
        self.zed.enable_object_detection(obj_param)

        if not (self.playback_svo and self.playback_filename != "") and self.record_svo and self.record_filename != "":
            recordingParameters = sl.RecordingParameters()
            recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
            recordingParameters.video_filename = self.record_filename
            self.get_logger().info(f"Starting to record the ZED data to an SVO named {recordingParameters.video_filename} with compression format {recordingParameters.compression_mode.name}")
            err = self.zed.enable_recording(recordingParameters)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to enable SVO recording on ZED. Error: {err}")

        self.get_logger().info(f"Finished initializing ZED Camera in {self.delta_time()} seconds")

    def init_visualizers(self):
        # Get Camera resolution
        camera_info = self.zed.get_camera_information()
        camera_res = camera_info.camera_configuration.resolution

        self.viewer = gl.GLViewer()
        self.viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, False)
    

    def cleanup(self):

        self.viewer.exit()
        # Disable modules and close camera
        self.zed.disable_recording()
        self.zed.disable_object_detection()
        self.zed.disable_positional_tracking()
        self.image_left_tmp.free(memory_type=sl.MEM.CPU)
        # self.image_for_display.free(memory_type=sl.MEM.CPU)
        
        self.zed.close()
        cv2.destroyAllWindows()

    def run_detections(self):
        self.record_timestamp()
        self.get_logger().info(f"Timer callback - Running zed.grab blocking call")

        # grab is a blocking call
        error_code = self.zed.grab(self.runtime_params)
        if error_code != sl.ERROR_CODE.SUCCESS:
            # TODO: Add handling for a camera error. Most likely error here is ERROR_CODE.CAMERA_NOT_DETECTED

            if error_code == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                self.get_logger().info(f"SVO end has been reached ({self.zed.get_svo_position()}). Looping back to first frame")
                self.zed.set_svo_position(0)

            return
        
        self.header_timestamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"zed.grab finished after {self.delta_time()} seconds")
        self.record_timestamp()

        detections: List[sl.CustomBoxObjectData] = []

        # ZED Image
        self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
        zed_left_image_net = self.image_left_tmp.get_data()
        zed_img = cv2.cvtColor(zed_left_image_net, cv2.COLOR_BGRA2BGR)

        # ZED Aruco Markers
        detections += self.detectVisionTargets.detectArucoMarkers(zed_img, CameraType.ZED)

        # ZED Red/Blue LEDS
        # detections += self.detectVisionTargets.detectZEDLEDs()

        if self.hsv_explore:
            # self.blue_led_processing.mask_step_tuning(zed_img)

            mask = self.blue_led_processing.process_mask(zed_img)
            bounding_boxes = self.blue_led_processing.process_contours(mask, zed_img)

            for i in range(0, len(bounding_boxes)):
                obj = sl.CustomBoxObjectData()
                obj.unique_object_id = f"blue_led_{i}"
                obj.bounding_box_2d = bounding_boxes[i]
                obj.label = 3
                obj.probability = 0.99
                obj.is_grounded = False
                detections.append(obj)

        # IR Cam Image
        # ir_image = self.ir_cam.read()

        # IR Cam Aruco Markers
        # detections += self.detectVisionTargets.detectArucoMarkers(ir_image, self.ir_cam.cam_type)

        # IR Cam LEDs
        # detections += self.detectVisionTargets.detectIRLEDS(ir_image, ir_cam.cam_type)

        # Ingest detections and get objects
        self.zed.ingest_custom_box_objects(detections)
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        if self.viewer.is_available():
            self.viewer.update_view(self.image_left_tmp, self.objects)

        # Iterate object detections
        zed_aruco_markers_msg = self.create_aruco_markers_msg()
        blue_led_point_arr = self.create_point_array()
        red_led_point_arr = self.create_point_array()
        ir_led_point_arr = self.create_point_array()

        for object in self.objects.object_list:
            self.get_logger().info(f"Object ~ Id: {object.id}, label: {object.label}, Unique Label: {object.unique_object_id}, position: {object.position}")
            DetectVisionTargets.draw_object_detection(zed_img, object)
            point: Point = self.zed_object_to_point(object)

            if DetectVisionTargets.is_zed_marker(object.unique_object_id):
                zed_aruco_markers_msg.points.append(point)
                zed_aruco_markers_msg.marker_ids.append(DetectVisionTargets.get_marker_id_from_label(object.unique_object_id))

                self.get_logger().info(f"Found Aruco: {zed_aruco_markers_msg}")

            elif (object.unique_object_id == "blue_led"):
                blue_led_point_arr.points.append(point)

            elif (object.unique_object_id == "red_led"):
                red_led_point_arr.points.append(point)

            elif (object.unique_object_id == "ir_led"):
                ir_led_point_arr.points.append(point)
        
        self.publish_zed_aruco_points.publish(zed_aruco_markers_msg)
        self.publish_blue_led_points.publish(blue_led_point_arr)
        self.publish_red_led_points.publish(red_led_point_arr)
        self.publish_ir_led_points.publish(ir_led_point_arr)

        self.get_logger().info(f"Zed computations finished in {self.delta_time()} seconds")

        cv2.imshow("ZED Image Processing", zed_img)
        cv2.waitKey(10)

    def create_aruco_markers_msg(self) -> ArucoMarkers:
        markers = ArucoMarkers()
        markers.header.frame_id = self.frame_id
        markers.header.stamp = self.header_timestamp
        return markers

    def create_point_array(self) -> PointArray:
        point_arr = PointArray()
        point_arr.header.frame_id = self.frame_id
        point_arr.header.stamp = self.header_timestamp
        return point_arr

    def zed_object_to_point(self, object: sl.ObjectData) -> Point:
        point = Point()
        point.x = object.position[0]
        point.y = object.position[1]
        point.z = object.position[2]
        return point

    def record_timestamp(self):
            self.timestamp = self.get_clock().now()

    def delta_time(self):
        return (self.get_clock().now() - self.timestamp).nanoseconds / 1000000000

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