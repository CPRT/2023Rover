from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ArmURDF2", package_name="moveit_urdf_v2").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
