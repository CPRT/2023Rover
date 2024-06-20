import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    parameters = [{
                "broadcast_utm_transform": "true"
    }]

    remappings = [
        ("imu/data", "imu"),
        ("gps/fix", "gps/fix"),
        ("odometry/filtered", "output from global ekf")
    ]

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            output="log",
            parameters=parameters,
            remappings=remappings,
            arguments=["--ros-args", "--log-level", "Warn"]),
    ])
    