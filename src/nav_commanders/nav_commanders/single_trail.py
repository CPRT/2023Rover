from enum import Enum
from typing import List, Optional
import math

from std_msgs.msg import Header, Time
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
    REQUEST_FLASH_ARUCO_CHECK = "RequestFlashArucoCheck"
    REQUEST_ARUCO_SCAN = "RequestArucoScan"
    BAKING_TARGET = "BakingTarget"

class TrailGoal(Enum):
    TWO_TARGET_GOAL = "TwoTargetGoal"
    ONE_TARGET_GOAL = "OneTargetGoal"
    SCANNING_GOAL = "ScanningGoal"
    RECOVERY_GOAL = "RecoveryGoal"
    NO_GOAL = "NoGoal"

class ReturnGoalAndStateAndPose:
    def __init__(self, pose: PoseStamped, goal: TrailGoal, state: TrailState):
        self.pose: PoseStamped = pose
        self.goal: TrailGoal = goal
        self.state: TrailState = state

class LogicTreeConstants:
    SAMPLES_TO_BAKE_TARGET = 10
    BAKING_OUTLIERS_THRESHOLD = 0.5 # meters

    MORE2_DISTANCE_THRESHOLD = 2.5 # meters
    ONE_DISTANCE_THRESHOLD = 2.5 # meters

    GOAL_COMPLETE_DISTANCE_THRESHOLD = 1.0 # meters

    FOLLOW_PREVIOUS_GOAL_TIMEOUT = 30.0 # seconds

    MORE2_FAR_DISTANCE_INFRONT = 1.5 # meters


    FRAME_ID = 'map'

class SingleCIRCTrail:
    def __init__(self, trail: TrailType, is_ir: bool):
        self._trail = trail
        self._trail_name = str(trail.value)
        self._is_ir = bool(is_ir)

        self._trail_state = TrailState.NONE_FOUND
        self._goal_state = TrailGoal.NO_GOAL
        self._prev_goal = PoseStamped()

        self._baked_targets: List[Point] = []
        self._curr_baking_points: List[Point] = []
    
    def run_led_trail(self, point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> TrailState:

        # TODO: EXLUCDE POINTS IN point_array TARE WITHIN THRESHOLD OF BAKED TARGETS


        # More than 2 targets
        if len(point_array) >= 2:
            TrailHelperFunctions.sort_point_array(point_array)

            if TrailHelperFunctions.distance_to_rover(point_array[0], rover_pose) < LogicTreeConstants.MORE2_DISTANCE_THRESHOLD:
                # Nearest target is close
                return self.two_targets_close(sorted_point_array=point_array, rover_pose=rover_pose, curr_time=curr_time)
            else:
                # Nearest target is far
                return self.two_targets_far(sorted_point_array=point_array, rover_pose=rover_pose, curr_time=curr_time)

        # Update whether the previous goal is still valid
        has_prev_goal = True
        if self._prev_goal != TrailGoal.NO_GOAL:
            # Check if the previous goal has timed out
            if TrailHelperFunctions.delta_time(curr_time, self._prev_goal.header.stamp) > LogicTreeConstants.FOLLOW_PREVIOUS_GOAL_TIMEOUT:
                self._prev_goal = TrailGoal.NO_GOAL
                has_prev_goal = False

            # If we've reached the previous goal, clear it
            distance_to_rover = TrailHelperFunctions.distance_to_rover(self._prev_goal, rover_pose)
            if distance_to_rover < LogicTreeConstants.GOAL_COMPLETE_DISTANCE_THRESHOLD:
                self._prev_goal = TrailGoal.NO_GOAL
                has_prev_goal = False

        # If the previous goal is valid and was from 2 targets, keep following it
        if has_prev_goal and self._goal_state == TrailGoal.TWO_TARGET_GOAL:
            return self._prev_goal

        # Only 1 target
        if len(point_array) == 1:
            if TrailHelperFunctions.distance_to_rover(point_array[0], rover_pose) < LogicTreeConstants.ONE_DISTANCE_THRESHOLD:
                # Close target
                return self.one_target_close(point_array, rover_pose, curr_time)
            else:
                # Far target
                return self.one_target_far(point_array, rover_pose, curr_time)

        # If the previous goal is valid and was from 1 target, keep following it
        if has_prev_goal and self._goal_state == TrailGoal.ONE_TARGET_GOAL:
            return self._prev_goal
        
        # No targets, do recovery

        # TODO: RECOVERY HERE POTENTIALLY

    def two_targets_far(self, sorted_point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> ReturnGoalAndStateAndPose:
        data = ReturnGoalAndStateAndPose()
        data.pose.header.frame_id = LogicTreeConstants.FRAME_ID
        data.pose.header.stamp = curr_time.to_msg()

        desired_yaw = TrailHelperFunctions.quaternion_from_yaw(TrailHelperFunctions.calc_angle_between_points(sorted_point_array[0], sorted_point_array[1]))
        
        data.pose.pose.position = sorted_point_array[0]
        data.pose.pose.orientation = TrailHelperFunctions.quaternion_from_yaw(desired_yaw)

        offset_point = Point()
        offset_point.x = LogicTreeConstants.MORE2_FAR_DISTANCE_INFRONT
        offset_point.y = 0
        TrailHelperFunctions.rotate_point_by_angle(desired_yaw + math.pi)
        TrailHelperFunctions.add_offset_to_pse(data.pose, offset_point)
        self._prev_goal = data.pose

        self._trail_state = TrailState.FOLLOWING_LEDS
        self._goal_state = TrailGoal.TWO_TARGET_GOAL
        data.state = self._trail_state
        data.goal = self._goal_state

        return data

    def two_targets_close(self, sorted_point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> TrailState:
        data = ReturnGoalAndStateAndPose()
        data.pose.header.frame_id = LogicTreeConstants.FRAME_ID
        data.pose.header.stamp = curr_time.to_msg()

        avg_point = TrailHelperFunctions.average_of_points(self._curr_baking_points)

        # If not enough samples, add the closest point
        if len(self._curr_baking_points) < LogicTreeConstants.SAMPLES_TO_BAKE_TARGET:
            self._curr_baking_points.append(sorted_point_array[0])

            # Remove outliers if we have enough samples
            if len(self._curr_baking_points) >= LogicTreeConstants.SAMPLES_TO_BAKE_TARGET - 2:
                self._curr_baking_points = TrailHelperFunctions.remove_point_outliers(self._curr_baking_points, LogicTreeConstants.BAKING_OUTLIERS_THRESHOLD, avg_point)

        # Check if we have enough samples to bake the target now
        if len(self._curr_baking_points) >= LogicTreeConstants.SAMPLES_TO_BAKE_TARGET:
            self._baked_targets.append(avg_point)
            self._trail_state = TrailState.FOLLOWING_LEDS
            self._curr_baking_points = []
        else:
            self._trail_state = TrailState.BAKING_TARGET

        desired_yaw = TrailHelperFunctions.quaternion_from_yaw(TrailHelperFunctions.calc_angle_between_points(sorted_point_array[0], sorted_point_array[1]))

        data.pose.pose.position = sorted_point_array[1]
        data.pose.pose.orientation = TrailHelperFunctions.quaternion_from_yaw(desired_yaw)

        offset_point = Point()
        offset_point.x = LogicTreeConstants.MORE2_FAR_DISTANCE_INFRONT
        offset_point.y = 0
        TrailHelperFunctions.rotate_point_by_angle(desired_yaw + math.pi)
        TrailHelperFunctions.add_offset_to_pse(data.pose, offset_point)
        self._prev_goal = data.pose

        self._goal_state = TrailGoal.TWO_TARGET_GOAL
        data.state = self._trail_state
        data.goal = self._goal_state

        return data



        


    def one_target_close(self, point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> TrailState:
        pass

    def one_target_far(self, point_array: PointArray, rover_pose: PoseStamped, curr_time: Time) -> TrailState:
        pass





class TrailHelperFunctions:
    def sort_point_array(self, point_array: PointArray) -> PointArray:
        point_array.points = point_array.points.sort(key = lambda p: (p.x**2 + p.y**2 + p.z**2))

    def distance_to_rover(self, point: Point, rover_pose: PoseStamped):
        return (point.x - rover_pose.position.x)**2 + (point.y - rover_pose.position.y)**2 + (point.z - rover_pose.position.z)**2
    
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

    def add_offset_to_pse(pose: PoseStamped, offset: Point):
        pose.position.x += offset.x
        pose.position.y += offset.y
        pose.position.z += offset.z

    def average_of_points(points: List[Point]) -> Point:
        avg = Point()
        avg.x = sum([p.x for p in points]) / len(points)
        avg.y = sum([p.y for p in points]) / len(points)
        avg.z = 0
        return avg
    
    def remove_point_outliers(points: List[Point], distance_from_avg: float, avg_point: Optional[Point] = None) -> List[Point]:
        """
        Removes points that are further than the threshold from the average of the points.
        """
        if avg_point is None:
            avg_point = TrailHelperFunctions.average_of_points(points)

        return [p for p in points if TrailHelperFunctions.distance_to_rover(p, avg_point) < distance_from_avg]