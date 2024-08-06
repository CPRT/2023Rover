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
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPose
from std_msgs.msg import Bool, Float32 
from interfaces.srv import NavToGPSGeopose
from interfaces.msg import ArucoMarkers, PointArray
from time import sleep
import math

from .single_trail import SingleCIRCTrail, TrailType, TrailState, TrailGoal, ReturnGoalAndStateAndPose, RecoverySteps, TrailHelperFunctions

class CIRCTrailsCommander(Node):
    """
    Class to do stuff yay. Please forget you read this.
    """

    def __init__(self):
        super().__init__('CIRCTrailsCommander')
        self.navigator = BasicNavigator("CIRCTrailsNavigator_Navigator")

        self.sub_rover_pose = self.create_subscription(Odometry, '/odometry/filtered/global', self.rover_pose_callback, 10)
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

        self.pub_goal = self.create_publisher(PoseStamped, '/circ_trails_goal', 10)
        self.pub_goal_yaw = self.create_publisher(Float32, '/circ_trails_goal_yaw', 10)

        self.pub_lights_on = self.create_publisher(Bool, '/lights_on', 10)

        self.blue_trail = SingleCIRCTrail(TrailType.BLUE_TRAIL, False, self.get_logger())
        self.red_trail = SingleCIRCTrail(TrailType.RED_TRAIL, False, self.get_logger())
        self.ir_trail = SingleCIRCTrail(TrailType.IR_TRAIL, True, self.get_logger())

        self.trails_in_order = [self.blue_trail, self.red_trail, self.ir_trail]
        self.trail_aruco_ids = [-1] * len(self.trails_in_order)
        self.trail_index = 0

        self.get_logger().info('Waiting for Nav2 to be active')
        # self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_timer_callback)
        self.get_logger().info("Starting the first trail named " + self.trails_in_order[0]._trail_name)

    def control_lights(self, is_lights_on: bool):
        msg = Bool()
        msg.data = is_lights_on
        self.pub_lights_on.publish(msg)

    def rover_pose_callback(self, msg: Odometry):
        self.last_rover_pose = PoseStamped()
        self.last_rover_pose.pose = msg.pose.pose
        self.last_rover_pose.header = msg.header

    def aruco_marker_callback(self, msg: ArucoMarkers):
        self.last_aruco_markers = msg
        self.last_arucos_used = False

    def blue_led_callback(self, msg: PointArray):
        self.last_blue_led = msg
        self.last_blue_used = False

    def red_led_callback(self, msg: PointArray):
        self.last_red_led = msg
        self.last_red_used = False

    def ir_led_callback(self, msg: PointArray):
        self.last_ir_led = msg
        self.last_ir_used = False

    def main_timer_callback(self):
        if self.trail_index >= len(self.trails_in_order):
            return

        if self.trails_in_order[self.trail_index]._trail == TrailType.BLUE_TRAIL:
            if not self.last_blue_used:
                self.last_blue_used = True
                led_points = self.last_blue_led
            else:
                return

        elif self.trails_in_order[self.trail_index]._trail == TrailType.RED_TRAIL:
            if not self.last_red_used:
                self.last_red_used = True
                led_points = self.last_red_led
            else:
                return

        elif self.trails_in_order[self.trail_index]._trail == TrailType.IR_TRAIL:
            if not self.last_ir_used:
                self.last_ir_used = True
                led_points = self.last_ir_led
            else:
                return

        else:
            return

        trail_success = self.run_trail(self.trails_in_order[self.trail_index], led_points)
        if trail_success:
            self.trail_index += 1
            if self.trail_index >= len(self.trails_in_order):
                self.get_logger().info("Finished all trails!")
            else:
                self.get_logger().info("Moving to next trail named " + str(self.trails_in_order[self.trail_index]._trail_name))


    def run_trail(self, trail: SingleCIRCTrail, led_points: PointArray) -> bool:
        data = trail.run_led_trail(led_points, self.last_rover_pose, self.get_clock().now())
        if data.recovery_step == RecoverySteps.FAILED_RECOVERY:
            raise Exception("Failed to recover from lack of trail data")
            
        # Run recovery
        if data.trail_goal_info in (TrailGoal.RECOVERY_GOAL, TrailGoal.NO_GOAL):
            data = trail.run_recovery(self.last_rover_pose, self.get_clock().now())

        self.control_lights(data.lights_on)
        self.pub_goal.publish(data.goal_pose)
        self.pub_goal_yaw.publish(Float32(data=math.degrees(TrailHelperFunctions.yaw_from_quaternion(data.goal_pose.pose.orientation))))
        self.get_logger().info(f"goal_state: {data.trail_goal_info.value}, trail_state: {data.trail_state_info.value}, recovery_step: {data.recovery_step.value} , lights_on: {data.lights_on}")

        # Scan for aruco markers
        if data.trail_state_info == TrailState.REQUEST_ARUCO_SCAN:
            found_aruco = trail.run_aruco_finder(self.last_aruco_markers, self.trail_aruco_ids, self.get_clock().now())
            if found_aruco != -1:
                self.trail_aruco_ids[self.trail_index] = found_aruco
                self.get_logger().info(f"~~~~\n~~~~~~~~~~~~~              Found aruco marker {found_aruco}, going to next trail type\n~~~~~~~~")
                return True
            
        # Stand still if trying to bake a target
        if data.trail_state_info == TrailState.BAKING_TARGET:
            self.get_logger().info("Baking target, standing still")
            # self.navigator.cancelTask()
            # Create a goal pose that is is the rover's current pose?
            return False
        
        # Set the desired goal pose
        # self.navigator.goToPose(data.goal_pose)
        return False


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
