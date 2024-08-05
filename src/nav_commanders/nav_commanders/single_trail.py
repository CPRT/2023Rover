from enum import Enum
from typing import List, Optional, Dict
import math
from numpy import arcsin, arccos, arctan2

from rclpy.time import Time
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion 
from interfaces.msg import PointArray, ArucoMarkers
from visualization_msgs.msg import MarkerArray, Marker

class TrailType(Enum):
    RED_TRAIL = "RedTrail"
    BLUE_TRAIL = "BlueTrail"
    IR_TRAIL = "IRTrail"

class TrailState(Enum):
    NONE_FOUND = "NoLEDsFound"
    FOLLOWING_LEDS = "FollowingLEDs"
    REQUEST_ARUCO_SCAN = "RequestArucoScan"
    SERACHING_LEDS = "SearchingLEDs"
    BAKING_TARGET = "BakingTarget"

class TrailGoal(Enum):
    TWO_TARGET_GOAL = "TwoTargetGoal"
    ONE_TARGET_GOAL = "OneTargetGoal"
    RECOVERY_GOAL = "RecoveryGoal"
    NO_GOAL = "NoGoal"

class RecoverySteps(Enum):
    NotRequired = "NotRequired"
    START_RECOVERY = "StartRecovery"
    FLASH_ARUCO_CHECK_FIRST = "FlashArucoCheck"
    FLASH_ARUCO_CHECK_SECOND = "FlashArucoCheckSecond"
    LED_CIRCLE_BACK_STRAIGHT = "CircleBackStraight"
    LED_CIRCLE_BACK_RIGHT = "CircleBackRight"
    LED_CIRCLE_BACK_LEFT = "CircleBackLeft"
    ARUCO_CIRCLE_BACK_STRAIGHT = "ArucoCircleBackStraight"
    ARUCO_CIRCLE_BACK_RIGHT = "ArucoCircleBackRight"
    ARUCO_CIRCLE_BACK_LEFT = "ArucoCircleBackLeft"
    FAILED_RECOVERY = "FailedRecovery"

class ReturnGoalAndStateAndPose:
    def __init__(self):
        self.goal_pose = PoseStamped()
        self.trail_goal_info = TrailGoal.NO_GOAL
        self.trail_state_info = TrailState.NONE_FOUND
        self.recovery_step = RecoverySteps.NotRequired
        self.lights_on = True

class LogicTreeConstants:
    FRAME_ID = 'map'

    # Trail following
    SAMPLES_TO_BAKE_TARGET = 10
    BAKING_OUTLIERS_THRESHOLD = 0.5 # meters
    IS_BAKED_TARGET_THRESHOLD_MOVING = 1.5 # meters
    IS_BAKED_TARGET_THRESHOLD_IDLE = 0.5 # meters

    MORE2_DISTANCE_THRESHOLD = 2.5 # meters
    ONE_DISTANCE_THRESHOLD = 2.5 # meters

    GOAL_COMPLETE_DISTANCE_THRESHOLD = 1.0 # meters

    FOLLOW_PREVIOUS_GOAL_TIMEOUT = 30.0 # seconds

    MORE2_FAR_DISTANCE_INFRONT = 1.5 # meters
    ONE_FAR_DISTANCE_INFRONT = 1.5 # meters
    ONE_CLOSE_DISTANCE_INFRONT = 0.3 # meters

    # Trail aruco finding
    ARUCO_DATA_TIMEOUT = 15.0 # seconds
    NUM_ARUCO_SAMPLES_REQUIRED = 10

    # Trail recoverey
    RECOVERY_GOAL_DISTANCE_COMPLETE_SMALL = 1.5 # meters
    RECOVERY_GOAL_DISTANCE_COMPLETE_LARGE = 3 # meters

    RECOVERY_FIRST_ARUCO_DISTANCE = 0.3 # meters
    RECOVERY_SECOND_ARUCO_DISTANCE = 2.0 # meters

    RECOVERY_STRAIGHT_ANGLE = 0.0 # radians
    RECOVERY_RIGHT_ANGLE = math.radians(40) # radians
    RECOVERY_LEFT_ANGLE = math.radians(-40) # radians

class SingleCIRCTrail:
    def __init__(self, trail: TrailType, is_ir: bool):
        self._trail = trail
        self._trail_name = str(trail.value)
        self._is_ir = bool(is_ir)

        self._aruco_finder = TrailArucoFinder(trail, is_ir)

        self._trail_state = TrailState.NONE_FOUND
        self._goal_state = TrailGoal.NO_GOAL
        self._prev_goal = PoseStamped()
        self._recovery_step = RecoverySteps.NotRequired
        self._has_recovery_goal = False
        self._recovery_goal = ReturnGoalAndStateAndPose()

        self._baked_targets: List[Point] = []
        self._curr_baking_points: List[Point] = []
    
    def run_led_trail(self, point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> ReturnGoalAndStateAndPose:
        new_points = PointArray()
        for i, p in enumerate(point_array.points):
            # Lock all points to the ground
            p.z = 0.0

            # Remove points that are previously baked
            for baked_point in self._baked_targets:
                if (not point_array.is_moving[i] and TrailHelperFunctions.distance_to_point(p, baked_point) > LogicTreeConstants.IS_BAKED_TARGET_THRESHOLD_IDLE) \
                        or (TrailHelperFunctions.distance_to_point(p, baked_point) > LogicTreeConstants.IS_BAKED_TARGET_THRESHOLD_MOVING):
                    new_points.points.append(p)
                    new_points.is_moving.append(point_array.is_moving[i])
                    break

        point_array = new_points

        # More than 2 targets
        if len(point_array.points) >= 2:
            TrailHelperFunctions.sort_point_array(point_array)

            if TrailHelperFunctions.distance_to_rover(point_array.points[0], rover_pose) < LogicTreeConstants.MORE2_DISTANCE_THRESHOLD:
                # Nearest target is close
                self.clear_recovery()
                return self.two_targets_close(sorted_point_array=point_array, rover_pose=rover_pose, curr_time=curr_time)
            else:
                # Nearest target is far
                self.clear_recovery()
                return self.two_targets_far(sorted_point_array=point_array, rover_pose=rover_pose, curr_time=curr_time)

        # Update whether the previous goal is still valid
        has_prev_goal = True
        if self._prev_goal != TrailGoal.NO_GOAL and self._goal_state != TrailGoal.RECOVERY_GOAL:
            # Check if the previous goal has timed out
            if TrailHelperFunctions.delta_time(curr_time, Time(self._prev_goal.header.stamp)) > LogicTreeConstants.FOLLOW_PREVIOUS_GOAL_TIMEOUT:
                self._prev_goal = TrailGoal.NO_GOAL
                has_prev_goal = False

            # If we've reached the previous goal, clear it
            distance_to_rover = TrailHelperFunctions.distance_to_rover(self._prev_goal, rover_pose)
            if distance_to_rover < LogicTreeConstants.GOAL_COMPLETE_DISTANCE_THRESHOLD:
                self._prev_goal = TrailGoal.NO_GOAL
                has_prev_goal = False

        # If the previous goal is valid and was from 2 targets, keep following it
        if has_prev_goal and self._goal_state == TrailGoal.TWO_TARGET_GOAL:
            self.clear_recovery()
            return self._prev_goal

        # Only 1 target
        if len(point_array.points) == 1:
            if TrailHelperFunctions.distance_to_rover(point_array[0], rover_pose) < LogicTreeConstants.ONE_DISTANCE_THRESHOLD:
                # Close target
                self.clear_recovery()
                return self.one_target_close(point_array, rover_pose, curr_time)
            else:
                # Far target
                self.clear_recovery()
                return self.one_target_far(point_array, rover_pose, curr_time)

        # If the previous goal is valid and was from 1 target, keep following it
        if has_prev_goal and self._goal_state == TrailGoal.ONE_TARGET_GOAL:
            self.clear_recovery()
            return self._prev_goal
        
        # No targets
        return self.no_targets()


    def bake_point(self, point: Point, is_moving: bool):
        if len(self._curr_baking_points) <= 1:
            avg_point = point
        else:
            avg_point = TrailHelperFunctions.average_of_points(self._curr_baking_points)

        # If not enough samples, add the closest point
        if len(self._curr_baking_points) < LogicTreeConstants.SAMPLES_TO_BAKE_TARGET and not is_moving:
            self._curr_baking_points.append(point)

            # Remove outliers if we have enough samples
            if len(self._curr_baking_points) >= LogicTreeConstants.SAMPLES_TO_BAKE_TARGET - 2:
                self._curr_baking_points = TrailHelperFunctions.remove_point_outliers(self._curr_baking_points, LogicTreeConstants.BAKING_OUTLIERS_THRESHOLD, avg_point)

        # Check if we have enough samples to bake the target now
        if len(self._curr_baking_points) >= LogicTreeConstants.SAMPLES_TO_BAKE_TARGET:
            self._baked_targets.append(avg_point)
            trail_state = TrailState.FOLLOWING_LEDS
            self._curr_baking_points = []
        else:
            trail_state = TrailState.BAKING_TARGET

        return trail_state

    def two_targets_far(self, sorted_point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> ReturnGoalAndStateAndPose:
        data = ReturnGoalAndStateAndPose()
        data.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
        data.goal_pose.header.stamp = curr_time.to_msg()

        desired_yaw = TrailHelperFunctions.quaternion_from_yaw(TrailHelperFunctions.calc_angle_between_points(sorted_point_array.points[0], sorted_point_array.points[1]))
        
        data.goal_pose.pose.position = sorted_point_array.points[0]
        data.goal_pose.pose.orientation = TrailHelperFunctions.quaternion_from_yaw(desired_yaw)

        offset_point = Point()
        offset_point.x = LogicTreeConstants.MORE2_FAR_DISTANCE_INFRONT
        offset_point.y = 0
        TrailHelperFunctions.rotate_point_by_angle(desired_yaw + math.pi)
        TrailHelperFunctions.add_offset_to_pose(data.goal_pose, offset_point)
        self._prev_goal = data.goal_pose

        self._trail_state = TrailState.FOLLOWING_LEDS
        self._goal_state = TrailGoal.TWO_TARGET_GOAL
        data.trail_state_info = self._trail_state
        data.trail_goal_info = self._goal_state

        data.lights_on = self._is_ir

        return data

    def two_targets_close(self, sorted_point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> ReturnGoalAndStateAndPose:
        data = ReturnGoalAndStateAndPose()
        data.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
        data.goal_pose.header.stamp = curr_time.to_msg()

        self._trail_state = self.bake_point(sorted_point_array.points[0], sorted_point_array.is_moving[0])

        desired_yaw = TrailHelperFunctions.calc_angle_between_points(sorted_point_array.points[0], sorted_point_array.points[1])

        data.goal_pose.pose.position = sorted_point_array.points[1]
        data.goal_pose.pose.orientation = TrailHelperFunctions.quaternion_from_yaw(desired_yaw)

        offset_point = Point()
        offset_point.x = LogicTreeConstants.MORE2_FAR_DISTANCE_INFRONT
        offset_point.y = 0.0
        TrailHelperFunctions.rotate_point_by_angle(offset_point, desired_yaw + math.pi)
        TrailHelperFunctions.add_offset_to_pose(data.goal_pose, offset_point)
        self._prev_goal = data.goal_pose

        self._goal_state = TrailGoal.TWO_TARGET_GOAL
        data.trail_state_info = self._trail_state
        data.trail_goal_info = self._goal_state

        data.lights_on = self._is_ir

        return data

    def one_target_far(self, point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> ReturnGoalAndStateAndPose:
        data = ReturnGoalAndStateAndPose()
        data.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
        data.goal_pose.header.stamp = curr_time.to_msg()

        if len(self._baked_targets) > 0:
            desired_yaw = TrailHelperFunctions.calc_angle_between_points(point_array.points[0], self._baked_targets[-1])
        else:
            desired_yaw = TrailHelperFunctions.calc_angle_between_points(point_array.points[0], rover_pose.pose.position)

        data.goal_pose.pose.position = point_array.points[0]
        data.goal_pose.pose.orientation = TrailHelperFunctions.quaternion_from_yaw(desired_yaw)

        offset_point = Point()
        offset_point.x = LogicTreeConstants.ONE_FAR_DISTANCE_INFRONT
        offset_point.y = 0.0
        TrailHelperFunctions.rotate_point_by_angle(offset_point, desired_yaw + math.pi)
        TrailHelperFunctions.add_offset_to_pose(data.goal_pose, offset_point)
        self._prev_goal = data.goal_pose

        self._trail_state = TrailState.FOLLOWING_LEDS
        self._goal_state = TrailGoal.ONE_TARGET_GOAL
        data.trail_state_info = self._trail_state
        data.trail_goal_info = self._goal_state

        data.lights_on = self._is_ir

        return data


    def one_target_close(self, point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> ReturnGoalAndStateAndPose:
        """
        Handle the case where there is only one target and it is close.

        Args:
            point_array: The point array message from the camera.
            rover_pose: The current pose of the rover.
            curr_time: The current time.

        Returns:
            The goal and state data.
        """
        data = ReturnGoalAndStateAndPose()
        data.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
        data.goal_pose.header.stamp = curr_time.to_msg()

        self._trail_state = self.bake_point(point_array.points[0], point_array.is_moving[0])

        if len(self._baked_targets) > 0:
            desired_yaw = TrailHelperFunctions.calc_angle_between_points(point_array.points[0], self._baked_targets[-1])
        else:
            desired_yaw = TrailHelperFunctions.calc_angle_between_points(point_array.points[0], rover_pose.pose.position)

        data.goal_pose.pose.position = point_array.points[0]
        data.goal_pose.pose.orientation = TrailHelperFunctions.quaternion_from_yaw(desired_yaw)

        offset_point = Point()
        offset_point.x = LogicTreeConstants.ONE_CLOSE_DISTANCE_INFRONT
        offset_point.y = 0.0
        TrailHelperFunctions.rotate_point_by_angle(offset_point, desired_yaw + math.pi)
        TrailHelperFunctions.add_offset_to_pose(data.goal_pose, offset_point)
        self._prev_goal = data.goal_pose

        self._goal_state = TrailGoal.ONE_TARGET_GOAL
        data.trail_state_info = self._trail_state
        data.trail_goal_info = self._goal_state

        data.lights_on = self._is_ir

        return data
    
    def no_targets(self):
        data = ReturnGoalAndStateAndPose()
        self._trail_state = TrailState.NONE_FOUND
        self._goal_state = TrailGoal.NO_GOAL

        data.trail_state_info = self._trail_state
        data.trail_goal_info = self._goal_state

        return data


    def clear_recovery(self):
        self._recovery_step = RecoverySteps.NotRequired
        self._recovery_goal = ReturnGoalAndStateAndPose()
        self._has_recovery_goal = False

    def run_recovery(self, rover_pose: PoseStamped, curr_time: Time) -> ReturnGoalAndStateAndPose:
        if self._recovery_step == RecoverySteps.NotRequired:
            data = ReturnGoalAndStateAndPose()
            data.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
            data.goal_pose.header.stamp = curr_time.to_msg()
            return data
        elif self._recovery_step == RecoverySteps.START_RECOVERY:
            self._recovery_goal = ReturnGoalAndStateAndPose()
            self._has_recovery_goal = False

        if self._has_recovery_goal:
            # Check if the recovery goal is within the goal complete distance threshold
            distance_to_goal = TrailHelperFunctions.distance_to_rover(self._recovery_goal.goal_pose.pose.position, rover_pose)

            # If we are close enough to the goal with a small distance, clear the recovery goal
            if distance_to_goal < LogicTreeConstants.RECOVERY_GOAL_DISTANCE_COMPLETE_SMALL:
                self._recovery_goal = ReturnGoalAndStateAndPose()
                self._has_recovery_goal = False

            # If we are close enough to the goal with a large distance for specific recovery steps, clear the recovery goal
            elif self._recovery_step != RecoverySteps.FLASH_ARUCO_CHECK_FIRST and self._recovery_step != RecoverySteps.FLASH_ARUCO_CHECK_SECOND and (
                    distance_to_goal < LogicTreeConstants.RECOVERY_GOAL_DISTANCE_COMPLETE_LARGE):
                self._recovery_goal = ReturnGoalAndStateAndPose()
                self._has_recovery_goal = False

            else:
                # Continue on our previous recovery goal
                self._has_recovery_goal = True
        
        if self._has_recovery_goal:
            return self._recovery_goal

        if self._recovery_step == RecoverySteps.START_RECOVERY:
            self._recovery_step = RecoverySteps.FLASH_ARUCO_CHECK_FIRST
            self._recovery_goal = ReturnGoalAndStateAndPose()
            self._has_recovery_goal = True

            self._recovery_goal.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
            self._recovery_goal.goal_pose.header.stamp = curr_time.to_msg()
            self._recovery_goal.goal_pose.pose.position = rover_pose.pose.position
            self._recovery_goal.goal_pose.pose.orientation = rover_pose.pose.orientation

            # Move the goal forward by a small distance
            rover_yaw = TrailHelperFunctions.yaw_from_quaternion(rover_pose.pose.orientation)
            offset_point = Point()
            offset_point.x = LogicTreeConstants.RECOVERY_FIRST_ARUCO_DISTANCE
            offset_point.y = 0.0
            TrailHelperFunctions.rotate_point_by_angle(offset_point, rover_yaw)
            TrailHelperFunctions.add_offset_to_pose(self._recovery_goal.goal_pose, offset_point)

            self._recovery_goal.trail_state_info = TrailState.REQUEST_ARUCO_SCAN
            self._recovery_goal.trail_goal_info = TrailGoal.RECOVERY_GOAL
            self._recovery_goal.recovery_step = self._recovery_step

            self._recovery_goal.lights_on = True
            
            return self._recovery_goal
        
        elif self._recovery_step == RecoverySteps.FLASH_ARUCO_CHECK_FIRST:
            self._recovery_step = RecoverySteps.FLASH_ARUCO_CHECK_SECOND
            self._recovery_goal = ReturnGoalAndStateAndPose()
            self._has_recovery_goal = True

            self._recovery_goal.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
            self._recovery_goal.goal_pose.header.stamp = curr_time.to_msg()
            self._recovery_goal.goal_pose.pose.position = rover_pose.pose.position
            self._recovery_goal.goal_pose.pose.orientation = rover_pose.pose.orientation

            # Move the goal forward by the second aruco distance
            rover_yaw = TrailHelperFunctions.yaw_from_quaternion(rover_pose.pose.orientation)
            offset_point = Point()
            offset_point.x = LogicTreeConstants.RECOVERY_SECOND_ARUCO_DISTANCE
            offset_point.y = 0.0
            TrailHelperFunctions.rotate_point_by_angle(offset_point, rover_yaw)
            TrailHelperFunctions.add_offset_to_pose(self._recovery_goal.goal_pose, offset_point)

            self._recovery_goal.trail_state_info = TrailState.REQUEST_ARUCO_SCAN
            self._recovery_goal.trail_goal_info = TrailGoal.RECOVERY_GOAL
            self._recovery_goal.recovery_step = self._recovery_step

            self._recovery_goal.lights_on = True

            return self._recovery_goal
        
        elif self._recovery_step == RecoverySteps.FLASH_ARUCO_CHECK_SECOND:
            self._recovery_step = RecoverySteps.LED_CIRCLE_BACK_STRAIGHT
            self.recovery_create_goal(curr_time, LogicTreeConstants.RECOVERY_STRAIGHT_ANGLE, True)

        elif self._recovery_step == RecoverySteps.LED_CIRCLE_BACK_STRAIGHT:
            self._recovery_step = RecoverySteps.LED_CIRCLE_BACK_LEFT
            self.recovery_create_goal(curr_time, LogicTreeConstants.RECOVERY_RIGHT_ANGLE, True)
        
        elif self._recovery_step == RecoverySteps.LED_CIRCLE_BACK_RIGHT:
            self._recovey_step = RecoverySteps.LED_CIRCLE_BACK_LEFT
            self.recovery_create_goal(curr_time, LogicTreeConstants.RECOVERY_LEFT_ANGLE, True)

        elif self._recovery_step == RecoverySteps.LED_CIRCLE_BACK_LEFT:
            self._recovery_step = RecoverySteps.ARUCO_CIRCLE_BACK_STRAIGHT
            self.recovery_create_goal(curr_time, LogicTreeConstants.RECOVERY_STRAIGHT_ANGLE, False)

        elif self._recovery_step == RecoverySteps.ARUCO_CIRCLE_BACK_STRAIGHT:
            self._recovery_step = RecoverySteps.ARUCO_CIRCLE_BACK_RIGHT
            self.recovery_create_goal(curr_time, LogicTreeConstants.RECOVERY_RIGHT_ANGLE, False)

        elif self._recovery_step == RecoverySteps.ARUCO_CIRCLE_BACK_RIGHT:
            self._recovery_step = RecoverySteps.ARUCO_CIRCLE_BACK_LEFT
            self.recovery_create_goal(curr_time, LogicTreeConstants.RECOVERY_LEFT_ANGLE, False)

        else:
            self._recovery_step = RecoverySteps.FAILED_RECOVERY
            self._recovery_goal = ReturnGoalAndStateAndPose()
            self._has_recovery_goal = True

            self._recovery_step = self._recovery_step
            self._recovery_goal.trail_state_info = TrailState.NONE_FOUND
            self._recovery_goal.trail_goal_info = TrailGoal.NO_GOAL

            self._recovery_goal.lights_on = True

        return self._recovery_goal

    def recovery_create_goal(self, curr_time: Time, yaw_offset: float, is_leds_not_aruco: bool):
        self._recovery_goal = ReturnGoalAndStateAndPose()
        self._has_recovery_goal = True

        self._recovery_goal.goal_pose.header.frame_id = LogicTreeConstants.FRAME_ID
        self._recovery_goal.goal_pose.header.stamp = curr_time.to_msg()
        self.return_to_last_baked_target(self._recovery_goal, yaw_offset)

        self._recovery_goal.trail_state_info = TrailState.SERACHING_LEDS if is_leds_not_aruco else TrailState.REQUEST_ARUCO_SCAN
        self._recovery_goal.trail_goal_info = TrailGoal.RECOVERY_GOAL
        self._recovery_goal.recovery_step = self._recovery_step
        self._recovery_goal.lights_on = self._is_ir if is_leds_not_aruco else True
        
    def return_to_last_baked_target(self, data: ReturnGoalAndStateAndPose, desired_yaw_offset_rad: float):
        if len(self._baked_targets) < 2:
            data.trail_state_info = TrailState.NONE_FOUND
            data.trail_goal_info = TrailGoal.NO_GOAL
            return data

        yaw_from_first_target = TrailHelperFunctions.calc_angle_between_points(self._baked_targets[0], self._baked_targets[-1])
        desired_yaw = yaw_from_first_target + desired_yaw_offset_rad

        data.goal_pose.pose.position = self._baked_targets[-1]
        data.goal_pose.pose.orientation = TrailHelperFunctions.quaternion_from_yaw(desired_yaw)

        return data

    def run_aruco_finder(self, aruco_markers: ArucoMarkers, curr_time: Time) -> int:
        return self._aruco_finder.run_aruco_finder(aruco_markers, curr_time)

class TrailArucoFinder:
    def __init__(self, trail: TrailType, is_ir: bool):
        self._trail = trail
        self._trail_name = str(trail.value)
        self._is_ir = bool(is_ir)

        self._previous_aruco_markers: List[ArucoMarkers] = []

    def run_aruco_finder(self, aruco_markers: ArucoMarkers, curr_time: Time) -> int:
        if len(aruco_markers.markers) == 0:
            return -1
        
        self._previous_aruco_markers.append(aruco_markers)
        new_markers: List[ArucoMarkers] = []
        for markers in self._previous_aruco_markers:
            # Only keep marker data that is not too old
            if TrailHelperFunctions.delta_time(curr_time, Time.from_msg(markers.header.stamp)) < LogicTreeConstants.ARUCO_DATA_TIMEOUT:
                new_markers.append(markers)

        self._previous_aruco_markers = new_markers

        # Count the number of times each marker id appears
        marker_id_counts: Dict[int, int] = {} # dict[marker_id] = count
        for marker_group in self._previous_aruco_markers:
            for marker in marker_group.markers:
                if not marker.id in marker_id_counts:
                    marker_id_counts[marker.id] = 1
                else:
                    marker_id_counts[marker.id] += 1

        # Find the marker with the most counts
        max_count = 0
        max_marker_id = -1
        for marker_id, count in marker_id_counts.items():
            if count > max_count:
                max_count = count
                max_marker_id = marker_id

        # If we have enough samples of the same marker, return the marker id
        if max_count >= LogicTreeConstants.NUM_ARUCO_SAMPLES_REQUIRED:
            return max_marker_id
        
        return -1

class TrailHelperFunctions:
    def sort_point_array(point_array: PointArray) -> PointArray:
        point_array.points.sort(key = lambda p: (p.x**2 + p.y**2))

    def distance_to_point(point1: Point, point2: Point):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def distance_to_rover(point: Point, rover_pose: PoseStamped):
        return TrailHelperFunctions.distance_to_point(point, rover_pose.pose.position)
    
    def delta_time(second_timestamp: Time, first_timestamp: Time) -> float:
        """
        Returns the difference in seconds between two timestamps.
        """
        return (second_timestamp - first_timestamp).nanoseconds / 1000000000
        
    def calc_angle_between_points(point1: Point, point2: Point) -> float:
        return math.atan2(point2.y - point1.y, point2.x - point1.x)
   
    def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.x = cy * cp * cr + sy * sp * sr
        q.y = cy * cp * sr - sy * sp * cr
        q.z = sy * cp * sr + cy * sp * cr
        q.w = sy * cp * cr - cy * sp * sr

        return q

    def quaternion_from_yaw(yaw: float) -> Quaternion:
        return TrailHelperFunctions.quaternion_from_euler(0, 0, yaw)
    
    def euler_from_quaternion(quaternion: Quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def yaw_from_quaternion(quaternion: Quaternion) -> float:
        return TrailHelperFunctions.euler_from_quaternion(quaternion)[2]

    def rotate_pose_by_angle(pose: PoseStamped, angle: float):
        """
        Rotates a pose by a given angle in radians.
        """
        pose.position.x = pose.position.x * math.cos(angle) - pose.position.y * math.sin(angle)
        pose.position.y = pose.position.x * math.sin(angle) + pose.position.y * math.cos(angle)

    def rotate_point_by_angle(point: Point, angle: float):
        """
        Rotates a point by a given angle in radians.
        """
        point.x = point.x * math.cos(angle) - point.y * math.sin(angle)
        point.y = point.x * math.sin(angle) + point.y * math.cos(angle)

    def add_offset_to_pose(pose: PoseStamped, offset: Point):
        pose.pose.position.x += offset.x
        pose.pose.position.y += offset.y

    def average_of_points(points: List[Point]) -> Point:
        avg = Point()
        avg.x = sum([p.x for p in points]) / len(points)
        avg.y = sum([p.y for p in points]) / len(points)
        avg.z = 0.0
        return avg
    
    def remove_point_outliers(points: List[Point], distance_from_avg: float, avg_point: Optional[Point] = None) -> List[Point]:
        """
        Removes points that are further than the threshold from the average of the points.
        """
        if avg_point is None:
            avg_point = TrailHelperFunctions.average_of_points(points)

        return [p for p in points if TrailHelperFunctions.distance_to_point(p, avg_point) < distance_from_avg]