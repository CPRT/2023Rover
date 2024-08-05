import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
from robot_localization.srv import FromLL
from rclpy.node import Node
from .gps_utils import latLonYaw2Geopose
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPose
from interfaces.srv import NavToGPSGeopose
from interfaces.msg import ArucoMarkers, PointArray
from time import sleep

from .single_trail import SingleCIRCTrail, TrailType, TrailState, TrailGoal, ReturnGoalAndStateAndPose

class CIRCTrailsCommander(Node):
    """
    Class to do stuff yay. Please forget you read this.
    """

    def __init__(self):
        super().__init__('CIRCTrailsCommander')
        self.navigator = BasicNavigator("CIRCTrailsNavigator_Navigator")

        self.sub_rover_pose = self.create_subscription(PoseStamped, '/rover_pose', self.rover_pose_callback, 10)
        self.last_rover_pose = PoseStamped()

        self.sub_aruco_markers = self.create_subscription(ArucoMarkers, '/zed/zed_aruco_points_map', self.aruco_marker_callback, 10)
        self.last_aruco_markers = ArucoMarkers()
        self.last_arucos_used = True

        self.sub_blue_led = self.create_subscription(PointArray, '/zed/blue_led_points_map', self.blue_led_callback, 10)
        self.last_blue_led = PointArray()
        self.last_blue_used = True

        self.sub_red_led = self.create_subscription(PointArray, '/zed/red_led_points_map', self.red_led_callback, 10)
        self.last_red_led = PointArray()
        self.last_red_used = True

        self.sub_ir_led = self.create_subscription(PointArray, '/zed/ir_led_points_map', self.ir_led_callback, 10)
        self.last_ir_led = PointArray()
        self.last_ir_used = True

        self.blue_trail = SingleCIRCTrail(TrailType.BLUE, False)

        self.get_logger().info('Waiting for Nav2 to be active')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_timer_callback)


    def rover_pose_callback(self, msg: PoseStamped):
        self.last_rover_pose = msg

    def aruco_marker_callback(self, msg: ArucoMarkers):
        self.last_aruco_markers = msg

    def blue_led_callback(self, msg: PointArray):
        self.last_blue_led = msg

    def red_led_callback(self, msg: PointArray):
        self.last_red_led = msg

    def ir_led_callback(self, msg: PointArray):
        self.last_ir_led = msg

    def main_timer_callback(self):
        """
        """
        if not self.last_blue_used:
            self.last_blue_used = True
            self.blue_trail.run_led_trail(self.last_blue_led, self.last_rover_pose, self.self.get_clock().now())


# def main(args=None):
#     rclpy.init(args=args)

#     node = GpsCommander()
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cleanup()

#     node.destroy_node()
#     rclpy.shutdown()


def main():
    rclpy.init()

    node = CIRCTrailsCommander()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
