import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""


    container = ComposableNodeContainer(
        name="PhoenixContainer",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="scienceBase",
                parameters=[{"id": 7},
                            {"P":5.0},
                            {"I":0.0},
                            {"D":0.0},
                            {"max_voltage": 12.0},
                            {"invert_sensor":True}],
                
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="digger",
                parameters=[{"id": 8},
                            {"P":100.0},
                            {"I":0.0},
                            {"D":0.0}],
                
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([container,
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joystick'),
        launch_ros.actions.Node(
            package='arm_interface',
            executable='joystick_science_controller',
            name='joystick_science_controller_node'),
        launch_ros.actions.Node(
            package='drive',
            executable='joystick_breakout',
            name='joystick_breakout_node'),
            
            
            ])
