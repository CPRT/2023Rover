import rclpy
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


class GpsCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        super().__init__('minimal_client_async')
        self.navigator = BasicNavigator("GpsCommander")

        self.nav_fix_topic = "/fromLL"
        self.geopose_service_name = "nav_to_gps_geopose"

        self.localizer = self.create_client(FromLL,  self.nav_fix_topic)
        self.geopose_service = self.create_service(GeoPose, self.geopose_service_name, self.geopose_server)

        while not self.localizer.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'Service on {self.nav_fix_topic} is not available, waiting again...')

        self.get_logger().info('Waiting for Nav2 to be active')
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        self.get_logger().info('Nav2 is active')

    def geopose_server(self, msg: NavToGPSGeopose, response: NavToGPSGeopose) -> NavToGPSGeopose:
        self.get_logger().info("Recieved a new gps goal")

        self.req = FromLL.Request()
        self.req.ll_point.longitude = msg.goal.position.longitude
        self.req.ll_point.latitude = msg.goal.position.latitude
        self.req.ll_point.altitude = msg.goal.position.altitude

        self.get_logger().info(f"Long{self.req.ll_point.longitude:.f}, Lat={self.req.ll_point.latitude:.f}, Alt={self.req.ll_point.altitude:.f}")

        self.future = self.localizer.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.pose.position = self.future.result().map_point

        self.get_logger().info(f"Target Pose in the map frame: x={self.future.result().map_point.x:.f}, y={self.future.result().map_point.y:.f}, z={self.future.result().map_point.z:.f}")

        self.target_pose.pose.orientation = msg.goal.orientation

        response.success = self.navigator.goToPose(self.target_pose)

        return response

def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "node"), "config", "demo_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()
