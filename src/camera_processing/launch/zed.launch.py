import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # LaunchConfigurations
    playback_filename = LaunchConfiguration('playback_filename')
    playback_filename_arg = DeclareLaunchArgument(
        'playback_filename',
        default_value='',
        description='When set to a non empty string, plays back the given .svo2 file as if the ZED was connected live.'
    )

    playback_start_index = LaunchConfiguration('playback_start_index')
    playback_start_index_arg = DeclareLaunchArgument(
        'playback_start_index',
        default_value='0',
        description='When playing back a .svo2 file, start at the given frame index.'
    )

    record_filename = LaunchConfiguration('record_filename')
    record_filename_arg = DeclareLaunchArgument(
        'record_filename',
        default_value='',
        description='When set to a non empty string, immediately to the given filename. Must have .svo2 suffix.'
    )

    publish_gl_viewer_data = LaunchConfiguration('publish_gl_viewer_data')
    publish_gl_viewer_data_args = DeclareLaunchArgument(
        'publish_gl_viewer_data',
        default_value='False',
        description='Options: True or False. When True, publishs additional data for a GLViewer node to display.'
    )

    publish_6x6_aruco_as_leds = LaunchConfiguration('publish_6x6_aruco_as_leds')
    publish_6x6_aruco_as_leds_args = DeclareLaunchArgument(
        'publish_6x6_aruco_as_leds',
        default_value='False',
        description="Options: True or False. " + 
                    "When True, aruco tags of the 6X6_50 family will be used to publish on the blue, red and ir topics " +
                    "(ID mapping: 0-15 = blue, 16-30 = red, 31-45 = ir)"
    )


    # Load YAML Parameters
    zed_params = os.path.join(
        get_package_share_directory('camera_processing'),
        'config',
        'zed_params.yaml'
    )

    colour_processing_params = os.path.join(
        get_package_share_directory('camera_processing'),
        'config',
        'colour_processing_params.yaml'
    )

    # Nodes
    zed_node = Node(
        package="camera_processing",
        executable="zed_node",
        name="zed",
        parameters=[
            zed_params, 
            colour_processing_params,
            {"playback_filename": playback_filename},
            {'record_filename': record_filename},
            {'publish_gl_viewer_data': publish_gl_viewer_data},
            {'publish_6x6_aruco_as_leds': publish_6x6_aruco_as_leds},
            {'playback_start_index': playback_start_index}
        ]
    )

    # Nodes 
    transform_zed_points_node = Node(
        package="camera_processing",
        executable="transform_zed_points_node",
        name="transform_zed_points"
    )

    # Add launch actions
    ld.add_action(playback_filename_arg)
    ld.add_action(playback_start_index_arg)
    ld.add_action(record_filename_arg)
    ld.add_action(publish_gl_viewer_data_args)
    ld.add_action(publish_6x6_aruco_as_leds_args)
    ld.add_action(zed_node)
    ld.add_action(transform_zed_points_node)
    return ld