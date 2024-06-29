from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/ttyACM0'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 10.0},
                #{'~ticks_per_meter': 4342.2},
                {'ticks_per_meter': 3802.4},
                {'ticks_per_rotation': 2780},
                {'base_width': 0.315},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': True},
            ]
        ),
        Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/ttyACM1'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 10.0},
                #{'~ticks_per_meter': 4342.2},
                {'ticks_per_meter': 3802.4},
                {'ticks_per_rotation': 2780},
                {'base_width': 0.315},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': True},
            ]
        )
    ])
