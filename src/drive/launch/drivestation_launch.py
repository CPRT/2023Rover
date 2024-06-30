import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription(
  [
        launch_ros.actions.Node(
            package='drive',
            executable='joystick_drive',
            name='joystick_drive_station'),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joystick'),
        launch_ros.actions.Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/ttyACM0'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 1.0},
                #{'~ticks_per_meter': 4342.2},
                {'ticks_per_meter': 3354},
                {'ticks_per_rotation': 4096},
                {'base_width': 0.81},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': False},
            ]
        ),
        launch_ros.actions.Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/ttyACM1'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 1.0},
                #{'~ticks_per_meter': 4342.2},
                {'ticks_per_meter': 3354},
                {'ticks_per_rotation': 4096},
                {'base_width': 0.81},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': False},
            ]
        ),
        launch_ros.actions.Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/ttyACM2'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 1.0},
                #{'~ticks_per_meter': 4342.2},
                {'ticks_per_meter': 3354},
                {'ticks_per_rotation': 4096},
                {'base_width': 0.81},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': False},
            ]
        )
  ]
  )
