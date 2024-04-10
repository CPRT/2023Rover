from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{"use_sim_time":True}]
    return LaunchDescription([
        Node(
            package='moveit_controller',
            parameters=parameters,
            executable='moveit_controller',
            name='moveit_controller'
        )
    ])
