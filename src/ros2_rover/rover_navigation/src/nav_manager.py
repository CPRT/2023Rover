import numpy as np
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from rover_msgs.msg import NavMode
from rover_msgs.msg import LightPath
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from robot_localization.srv import FromLL
from enum import Enum
from scipy.spatial.transform import Rotation


class NavManager(Node):

    class Mode(Enum):
        DRIVER_MODE = 0
        GPS_MODE = 1
        BLUE_PATH_MODE = 2
        RED_PATH_MODE = 3
        IR_PATH_MODE = 4

    
    def __init__(self):
        # set up subscriptions
        self.mode_sub = self.create_subscription(NavMode, "nav_system/mode", self.change_mode, 10)
        self.mode = DRIVER_MODE
        # set up publisher
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        # set up lat long to map service
        self.fromLL_client = self.create_client(FromLL, 'from_LL')
        while not self.fromLL_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FromLL service not available, waiting again...')
        self.fromLL_req = FromLL.Request()
    
    def change_mode(self, msg):
        self.mode = msg.mode
        if self.mode == DRIVER_MODE:
            self.sub = None
            return
        if self.mode == GPS_MODE
            self.sub = self.create_subscription(GeoPoint, "gps/topic", self.gps_callback)
            return
        if self.mode == BLUE_PATH_MODE
            self.sub = self.create_subscription(LightPath, "blue/light_path", self.color_callback, 10)
            return
        if self.mode == RED_PATH_MODE
            self.sub = self.create_subscription(LightPath, "red/light_path", self.color_callback, 10)
            return
        if self.mode == IR_PATH_MODE
            self.sub = self.create_subscription(LightPath, "ir/light_path", self.color_callback, 10)
            return
        self.get_logger().warning('W')


    def gps_callback(self, msg):
        self.fromLL_req.ll_point=msg
        future = self.fromLL_client.call_async(self.fromLL_req)
        rclpy.spin_until_future_complete(self, future)
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position = future.result()
        (transform, result) = get_current_pose("map")
        if result:
            msg.pose.orientation = calculate_orientation(transform.translation, msg.pose.position)
        send_point(msg)

    def color_callback(self, msg):
        frame = msg.header.frame_id
        if len(msg.data) = 0:
            return
        points = []
        for i in range(0, len(msg.data), 2):
            points.append([msg.data[i].x, msg.data[i+1].y])
        frame = msg.header.frame_id
        points.sort(key=distance_from_rover)
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.points[0].x
        msg.pose.position.y = self.points[0].y
        if (len(self.points) > 1):
            quaternion = calculate_orientation(self.points[0], self.points[1])
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
        elif len(self.points) == 1:
            # Use the current orientation of the rover
            (transform, result) = get_current_pose(frame)
            if result:
                msg.pose.orientation = transform.rotation
        send_point(msg)
        

    def send_point(self, pose):
        self.get_logger().info('Sending goal:\n x:{pose.pose.position.x}\n y:{pose.pose.position.x}')
        self.goal_pub.publish(pose)


def get_current_pose(target_frame, start_frame="base_link"):
    try:
        transfrom = self.tf_buffer.lookup_transform(start_frame, target_frame, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        self.get_logger().warning(f'Could not get transform from {self.frame} to base_link', once=True)
        return (None, False)
    return (transform.transform, True)


# calculates the orientation for the robot to point at point2 while at point1
def calculate_orientation(point1, point2):
    # Calculate direction vector from point 1 to point 2
    direction_vector = np.array([point2.x - point1.x, point2.y - point1.y])
    
    # Normalize the direction vector
    norm = np.linalg.norm(direction_vector)
    direction_vector = direction_vector / norm if norm != 0 else np.zeros_like(direction_vector)
    
    # Calculate the angle between the direction vector and the positive x-axis
    angle = np.arctan2(direction_vector[1], direction_vector[0])
    
    # Construct quaternion representing rotation around the z-axis by the angle
    quaternion_orientation = Rotation.from_euler('z', angle).as_quat()
    # returns an array in the from [x, y, z, w]
    return quaternion_orientation

def distance_from_rover(point):
        try:
            transfrom = self.tf_buffer.lookup_transform("base_link", self.frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.node.get_logger().warning(f'Could not get transform from {self.frame} to base_link', once=True)
            return 0
        transfrom = self.tf_buffer.lookup_transform("base_link", self.frame, rospy.Time())
        x = point.x + transform.transform.translation.x
        y = point.y + transform.transform.translation.y
        return sqrt(x**2 + y**2)