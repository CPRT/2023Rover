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
from std_msgs.msg import Bool
from interfaces.srv import NavToGPSGeopose
from interfaces.msg import ArucoMarkers, PointArray
from time import sleep

from .single_trail import SingleCIRCTrail, TrailType, TrailState, TrailGoal, ReturnGoalAndStateAndPose, RecoverySteps

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

        self.pub_goal = self.create_publisher(PoseStamped, '/circ_trails_goal', 10)

        self.pub_lights_on = self.create_publisher(Bool, '/lights_on', 10)

        self.blue_trail = SingleCIRCTrail(TrailType.BLUE_TRAIL, False)
        self.red_trail = SingleCIRCTrail(TrailType.RED_TRAIL, False)
        self.ir_trail = SingleCIRCTrail(TrailType.IR_TRAIL, True)

        self.trails_in_order = [self.blue_trail, self.red_trail, self.ir_trail]
        self.trail_index = 0

        self.get_logger().info('Waiting for Nav2 to be active')
        # self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_timer_callback)

    def control_lights(self, is_lights_on: bool):
        msg = Bool()
        msg.data = is_lights_on
        self.pub_lights_on.publish(msg)

    def rover_pose_callback(self, msg: PoseStamped):
        self.last_rover_pose = msg

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

        trail_success = self.trails_in_order[self.trail_index].run_led_trail(led_points, self.last_rover_pose, self.get_clock().now())
        if trail_success:
            self.trail_index += 1
            if self.trail_index >= len(self.trails_in_order):
                self.get_logger().info("Finished all trails!")


    def run_trail(self, trail: SingleCIRCTrail, led_points: PointArray) -> bool:
        """
        """
        data = trail.run_led_trail(led_points, self.last_rover_pose, self.get_clock().now())
        self.control_lights(data.is_lights_on)
        if data.recovery_step == RecoverySteps.FAILED_RECOVERY:
            raise Exception("Failed to recover from lack of trail data")
            
        # Run recovery
        if data.goal_state_info == TrailGoal.RECOVERY_GOAL or data.goal_state_info == TrailGoal.NO_GOAL:
            data = trail.run_recovery(self.last_rover_pose, self.get_clock().now())

        self.get_logger().info(f"goal_state: {data.trail_goal_info.value}, trail_state: {data.trail_state_info.value}, recovery_step: {data.recovery_step.value} , lights_on: {data.is_lights_on}")
        self.pub_goal.publish(data.goal_pose.pose)

        # Scan for aruco markers
        if data.trail_state_info == TrailState.REQUEST_ARUCO_SCAN:
            found_aruco = trail.run_aruco_finder(self.last_aruco_markers, self.get_clock().now())
            if found_aruco != -1:
                self.get_logger().info(f"Found aruco marker {found_aruco}, going to next trail type")
                return True
            
        # Stand still if trying to bake a target
        if data.trail_state_info == TrailState.BAKE_TARGET:
            self.get_logger().info("Baking target, standing still")
            # Create a goal pose that is is the rover's current pose?
            return False
        
        # Set the desired goal pose
        self.navigator.goToPose(data.goal_pose)
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
