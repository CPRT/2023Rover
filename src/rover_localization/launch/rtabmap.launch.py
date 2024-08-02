# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Whether to launch rtabmapviz")

    parameters = [{
        "frame_id": "base_link",
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "subscribe_scan_cloud": True,
        "approx_sync": True,
        "publish_tf": False,
        "use_sim_time": use_sim_time,
        "qos_imu": 2,

        "Optimizer/Strategy": "1",                  # 0=TORO, 1=g2o, 2=GTSAM
        "Optimizer/GravitySigma": "0.3",

        "RGBD/OptimizeMaxError": "1.0",
        "RGBD/OptimizeFromGraphEnd": "true",

        "GFTT/MinDistance": "2.5",
        "GFTT/QualityLevel": "0.1",

        "Vis/CorGuessWinSize": "40",
        "Vis/CorType": "0",
        "Vis/MaxFeatures": "1000",
        "Vis/MinDepth": "0.0",
        "Vis/MaxDepth": "2.5",

        "Grid/DepthDecimation": "2",
        "Grid/RangeMin": "1.0",
        "Grid/RangeMax": "2.5",
        "Grid/MinClusterSize": "20",
        "Grid/MaxGroundAngle": "35",
        "Grid/NormalK": "20",
        "Grid/CellSize": "0.2",
        "Grid/FlatObstacleDetected": "false",
        #"Grid/Sensor": "True",

        "GridGlobal/UpdateError": "0.01",
        "GridGlobal/MinSize": "200",

        "Reg/Strategy": "1"
    }]

    remappings = [
        ("scan_cloud", "ouster/points"),
        ("rgb/camera_info", "camera/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", "ouster/imu"),
        ("odom", "odometry/filtered/local"),
        ("goal", "goal_pose"),
        ("map", "map"),
    ]

    return LaunchDescription([
        use_sim_time_cmd,
        launch_rtabmapviz_cmd,

        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            output="screen",
            parameters=parameters,
            remappings=remappings,
            arguments=["-d", "--delete_db_on_start",
                       "--ros-args", "--log-level", "Warn"]),

        Node(
            condition=IfCondition(launch_rtabmapviz),
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings),
    ])