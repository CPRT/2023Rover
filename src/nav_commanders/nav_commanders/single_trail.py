from enum import Enum
from typing import List

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, PoseStamped
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

class SingleCIRCTrail:
    def __init__(self, trail: TrailType, is_ir: bool):
        self._trail = trail
        self._trail_name = str(trail.value)
        self._is_ir = bool(is_ir)
        self._trail_state = TrailState.NONE_FOUND

        self._baked_targets: List[Point] = []
    
    def run_led_trail(self, point_array: PointArray) -> TrailState:
        if len(point_array) >= 2:
            TrailHelperFunctions.sort_point_array(point_array)
            self.two_targets(point_array)
        elif len(point_array) == 1:
            self.one_target()


    def two_targets(self, point_array: PointArray) -> TrailState:
        pass







class TrailHelperFunctions:
    def sort_point_array(self, point_array: PointArray) -> PointArray:
        point_array.points = point_array.points.sort(key = lambda p: (p.x**2 + p.y**2 + p.z**2))

    def distance_to_rover(self, point: Point, rover_pose: PoseStamped):
        return (p.x - rover_pose.position.x)**2 + (p.y - rover_pose.position.y)**2 + (p.y - rover_pose.position.y)**2
   

    
