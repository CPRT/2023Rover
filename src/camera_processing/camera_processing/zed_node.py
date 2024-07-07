#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import pyzed.sl as sl

from rclpy.node import Node
from rclpy import Parameter
# import rclpy.time as Time
from typing import List
from threading import Lock, Thread
from time import sleep

from cv_bridge import CvBridge

from geometry_msgs.msg import Point
from interfaces.msg import PointArray, ArucoMarkers
from sensor_msgs.msg import CompressedImage, Image

from .zed_viewers.cv_viewer import tracking_viewer as cv_viewer
from .zed_viewers.ogl_viewer import viewer as gl

from .zed_helper_files.detect_vision_targets import DetectVisionTargets, CameraType
from .zed_helper_files.video_capture import VideoCapture
from .HSVImageExplore.image_colour_processing.colour_processing import ColourProcessing
from .HSVImageExplore.image_colour_processing.hsv_range_mask_step import HSVRangeMaskStep
from .HSVImageExplore.image_colour_processing.simple_mask_steps import ErodeDilateStep
from .HSVImageExplore.image_colour_processing.datatypes import HSVRange, HSV

class ZedNode(Node):
    STRING_TO_RESOLUTION = {"HD720": sl.RESOLUTION.HD720, "HD1080": sl.RESOLUTION.HD1080, "HD1200": sl.RESOLUTION.HD1200, "HD2K": sl.RESOLUTION.HD2K}

    def __init__(self):
        super().__init__('zed')
        self.setup_params()
        self.setup_detect_vision_targets()

        self.cv_bridge = CvBridge()

        if self.should_detect_ir_led:
            self.ir_cam: VideoCapture = VideoCapture(0, CameraType.ERIK_ELP)

        zed_initialized = False
        while not zed_initialized:
            if self.init_zed():
                zed_initialized = True
            else:
                sl.Camera.reboot(sn=0, full_reboot=True) # Reboot fixes USB problems on Jetson Nano
    
        self.init_visualizers()

        self.image_left_tmp: sl.Mat = sl.Mat()
        self.objects: sl.Object = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.runtime_params = sl.RuntimeParameters()

        self.publish_zed_aruco_points = self.create_publisher(ArucoMarkers, '/zed_aruco_points', 10)
        self.publish_blue_led_points = self.create_publisher(PointArray, '/blue_led_points', 10)
        self.publish_red_led_points = self.create_publisher(PointArray, '/red_led_points', 10)
        self.publish_ir_led_points = self.create_publisher(PointArray, '/ir_led_points', 10)

        self.publish_raw_image = self.create_publisher(Image, '/zed_raw_image', 10)
        self.publish_cv_image = self.create_publisher(CompressedImage, '/cv_zed_image', 10)

        self.timer_period = 0.03  # 0.066 for 15 FPS
        self.timer = self.create_timer(self.timer_period, self.run_detections)

        self.timestamp = self.get_clock().now()
        self.header_timestamp = self.get_clock().now().to_msg()

    def setup_params(self) -> bool:
        self.frame_id = "/zed_link"

        self.declare_parameters(
            namespace="",
            parameters=[ 
                # From zed_params.yaml. Below values are defaults, see zed_params.yaml for actual values
                ('zed_arucos_detections', True),
                ('blue_led_detections', True),
                ('red_led_detections', True),
                ('ir_led_detections', True),

                ('publish_raw_image', False),
                ('publish_cv_processed_image', False),
                ('svo_realtime_mode', False), # Doesn't work on Jetson Nano
                ('always_record', False),
                ('default_record_filename', ''),
                ('resolution', 'HD1080'), # Options: HD720, HD1080, HD1200, HD2K
                ('fps', 30),
                ('depth_maximum_distance', 15), # meters
                ('exposure', 50),
                ('gain', 95),
                ('gamma', 7),
                ('white_balance', 4600),
                ('enable_object_tracking', False),

                # From colour_processing_params.yaml. This yaml file is required
                # ('resize_for_processing', Parameter.Type.DOUBLE),
                # ('blue_led', Parameter.Type.STRING),
                ('red_led', Parameter.Type.STRING),
                # ('ir_led', Parameter.Type.STRING),
                ('resize_for_processing', 1.0),
                ('blue_led', ""),
                # ('red_led', ""),
                ('ir_led', ""),

                # From launch file arguments
                ('playback_filename', ''),
                ('record_filename', ''),
                ('publish_gl_viewer_data', False),
                ('publish_6x6_aruco_as_leds', False),
            ]
        )

        if self.get_parameter('resolution').value not in ZedNode.STRING_TO_RESOLUTION:
            raise ValueError(f"ROS2 parameter resolution in ZED Node is not one of {ZedNode.STRING_TO_RESOLUTION.keys()}")

        self.should_detect_arucos = bool(self.get_parameter('zed_arucos_detections').value)
        self.should_detect_blue_led = bool(self.get_parameter('blue_led_detections').value)
        self.should_detect_red_led = bool(self.get_parameter('red_led_detections').value)
        self.should_detect_ir_led = bool(self.get_parameter('ir_led_detections').value)

        self.should_publish_raw_image = bool(self.get_parameter('publish_raw_image').value)
        self.should_publish_cv_processed_image = bool(self.get_parameter('publish_cv_processed_image').value)
        self.should_publish_gl_viewer_data = bool(self.get_parameter('publish_gl_viewer_data').value)
        self.should_publish_6x6_aruco_as_leds = bool(self.get_parameter('publish_6x6_aruco_as_leds').value)

        self.record_svo: bool = False
        self.playback_svo: bool = False
        self.record_filename: str = ""
        self.playback_filename: str = ""

        if str(self.get_parameter('playback_filename').value) != '':
            self.playback_svo = True
            self.playback_filename = str(self.get_parameter('playback_filename').value)

        elif str(self.get_parameter('record_filename').value) != '':
            self.record_svo = True
            self.record_filename = str(self.get_parameter('record_filename').value)

        elif self.get_parameter('always_record') and str(self.get_parameter('default_record_filename')) != '':
            self.record_svo = True
            self.record_filename = str(self.get_parameter('default_record_filename'))

        self.blue_led_str = str(self.get_parameter('blue_led').value)
        self.red_led_str = str(self.get_parameter('red_led').value)
        self.ir_led_str = str(self.get_parameter('ir_led').value)
    
    def setup_detect_vision_targets(self):
        """
        Setup the DetectVisionTargets object with the ColourProcessing objects for the blue, red and ir LEDs.
        """
        blue_led_colour_processing = ColourProcessing.from_string(self.blue_led_str)
        red_led_colour_processing = ColourProcessing.from_string(self.red_led_str)
        ir_led_colour_processing = ColourProcessing.from_string(self.ir_led_str)

        if not blue_led_colour_processing or isinstance(blue_led_colour_processing, str):
            self.get_logger().error(f"Failed to create blue_led_colour_processing: {blue_led_colour_processing}") 
            self.should_detect_blue_led = False

        if not red_led_colour_processing or isinstance(red_led_colour_processing, str):
            self.get_logger().error(f"Failed to create red_led_colour_processing: {red_led_colour_processing}")
            self.should_detect_red_led = False

        if not ir_led_colour_processing or isinstance(ir_led_colour_processing, str):
            self.get_logger().error(f"Failed to create ir_led_colour_processing: {ir_led_colour_processing}")
            self.should_detect_ir_led = False

        self.detectVisionTargets = DetectVisionTargets(
            blue_led=blue_led_colour_processing,
            red_led=red_led_colour_processing,
            ir_led=ir_led_colour_processing
        )

    def init_zed(self) -> bool:
        """
        Initialize the ZED Camera. 
        Returns False when the camera should be rebooted and initialized again.

        Raises:
            Exception: Various exceptions from the ZED SDK about the camera failing to initialize.
        Returns:
            bool: True means the camera was correctly initialized. False means the camera should be rebooted and re-initialized.
        """
        self.record_timestamp()
        self.get_logger().info("Initializing ZED Camera")

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters(svo_real_time_mode=False)
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Can use PERFORMANCE MODE but it misses some details (sl.DEPTH_MODE.ULTRA) (sl.DEPTH_MODE.PERFORMANCE)
        init_params.depth_maximum_distance = int(self.get_parameter('depth_maximum_distance').value)
        init_params.camera_resolution = ZedNode.STRING_TO_RESOLUTION[str(self.get_parameter('resolution').value)]   # HD720   HD1080   HD1200    HD2K
        init_params.camera_fps = int(self.get_parameter('fps').value) # Use 15 FPS to improve low-light performance

        if self.playback_svo and self.playback_filename != "":
            init_params.set_from_svo_file(self.playback_filename)
            self.get_logger().info(f"Playing back SVO. Data from the ZED is a recording and NOT LIVE. Filename: {self.playback_filename}")
        
        self.zed = sl.Camera()
        status = self.zed.open(init_params)

        if status != sl.ERROR_CODE.SUCCESS:
            if status == sl.ERROR_CODE.CAMERA_NOT_DETECTED:
                self.get_logger().error("Failed to zed.open(init_params) Got ZED error code: " + repr(status))
                return False
        
        self.get_logger().info(f"white_balance: {int(self.get_parameter('white_balance').value)}")
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, int(self.get_parameter('exposure').value))
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, int(self.get_parameter('white_balance').value))
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 0 if int(self.get_parameter('white_balance').value) == -1 else 1)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, int(self.get_parameter('gain').value))
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, int(self.get_parameter('gamma').value))

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS  # CUSTOM_BOX_OBJECTS  MULTI_CLASS_BOX_MEDIUM
        obj_param.enable_tracking = bool(self.get_parameter('enable_object_tracking').value)
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
        return True

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
            if error_code == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                self.get_logger().info(f"SVO end has been reached ({self.zed.get_svo_position()}). Looping back to first frame")
                self.zed.set_svo_position(0)
            else:
                self.get_logger().error(f"ERROR_CODE reported from ZED from grab function. ERROR_CODE: {repr(error_code)}")
            return
        
        self.header_timestamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"zed.grab finished after {self.delta_time()} seconds")
        self.record_timestamp()

        detections: List[sl.CustomBoxObjectData] = []

        # ZED Image
        self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
        zed_left_image_net: np.array = self.image_left_tmp.get_data()
        zed_img = cv2.cvtColor(zed_left_image_net, cv2.COLOR_BGRA2BGR)

        # Publish Raw Image
        if self.should_publish_raw_image:
            self.publish_raw_image.publish(self.cv_bridge.cv2_to_imgmsg(zed_img))

        # ZED Aruco Markers
        if self.should_detect_arucos:
            detections += self.detectVisionTargets.detectArucoMarkers(zed_img, CameraType.ZED)

        # ZED Blue LEDS
        if self.should_detect_blue_led:
            detections += self.detectVisionTargets.detectZEDBlueLEDs(zed_img)

        # ZED Red LEDS
        if self.should_detect_red_led:
            detections += self.detectVisionTargets.detectZEDRedLEDs(zed_img)

        # IR Cam Image 
        if self.should_detect_ir_led:
            ir_image = self.ir_cam.read()

            # IR Cam Aruco Markers
            detections += self.detectVisionTargets.detectArucoMarkers(ir_image, self.ir_cam.cam_type)

            # IR Cam LEDs
            detections += self.detectVisionTargets.detectIRLEDS(ir_image, self.ir_cam.cam_type)

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

        if self.should_publish_cv_processed_image:
            self.publish_cv_image.publish(self.cv_bridge.cv2_to_compressed_imgmsg(zed_img)) 

        # cv2.imshow("ZED Image Processing", zed_img)
        # cv2.waitKey(10)

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