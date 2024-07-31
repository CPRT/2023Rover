from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{"use_sim_time":False}]
    return LaunchDescription([
        Node(
            package='current_pose_tester',
            parameters=parameters,
            executable='current_pose_tester',
            name='current_pose_tester'
        )
    ])
