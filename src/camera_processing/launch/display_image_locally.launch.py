from launch import LaunchDescription  
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # LaunchConfigurations
    window_name = LaunchConfiguration('window_name')
    window_name_arg = DeclareLaunchArgument(
        'window_name',
        default_value='DisplayImage',
        description="The window name that's displaying the image."
    )

    image_topic = LaunchConfiguration('image_topic')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/zed/cv_zed_image',
        description='The name of the topic to subscribe and get the image from.'
    )

    is_image_compressed = LaunchConfiguration('is_image_compressed')
    is_image_compressed_args = DeclareLaunchArgument(
        'is_image_compressed',
        default_value='False',
        description='Options: True or False. True to expect a sensor_msgs.msg.CompressedImage, False to expect a sensor_msgs.msg.Image'
    )

    # Nodes
    node = Node(
        package="camera_processing",
        executable="display_image_locally",
        name="display_image_locally",
        parameters=[
            {"window_name": window_name},
            {'image_topic': image_topic},
            {'is_image_compressed': is_image_compressed},
        ]
    )

    ld.add_action(window_name_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(is_image_compressed_args)
    ld.add_action(node)
    return ld