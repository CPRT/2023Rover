import os
 
import ament_index_python.packages
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    default_params_file = os.path.join(get_package_share_directory(
        "rover_localization"), "config", "gps.yaml")

    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')
    
    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[params_file])
 
    return launch.LaunchDescription([ublox_gps_node,
                                     params_file_arg,
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
    