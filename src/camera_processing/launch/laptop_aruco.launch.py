import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # main_param_dir = launch.substitutions.LaunchConfiguration(
    #     'main_param_dir',
    #     default=os.path.join(
    #         get_package_share_directory('rover_localization'),
    #         'config',
    #         'lidarslam.yaml'))
    
    # rviz_param_dir = launch.substitutions.LaunchConfiguration(
    #     'rviz_param_dir',
    #     default=os.path.join(
    #         get_package_share_directory('lidarslam'),
    #         'rviz',
    #         'mapping.rviz'))

    # mapping = launch_ros.actions.Node(
    #     package='scanmatcher',
    #     executable='scanmatcher_node',
    #     parameters=[main_param_dir],
    #     remappings=[('/input_cloud','/ouster/points'),
    #                 ('/current_pose','/slam/pose'),
    #                 ('/imu','/ouster/imu')],
    #     output='screen'
    # )


    # graphbasedslam = launch_ros.actions.Node(
    #     package='graph_based_slam',
    #     executable='graph_based_slam_node',
    #     parameters=[main_param_dir],
    #     output='screen'
    # )
    

    aruco_node = launch_ros.actions.Node(
        package='camera_processing',
        executable='aruco_markers',
        parameters=[{"is_image_compressed": True}],
        remappings=[('/source_image', '/camera1/image_compressed')],
        name='laptop_aruco'
    )
    


    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(
        #     'main_param_dir',
        #     default_value=main_param_dir,
        #     description='Full path to main parameter file to load'),
        aruco_node
    ])