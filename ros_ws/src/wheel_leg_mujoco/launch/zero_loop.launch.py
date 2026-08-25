from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model = PathJoinSubstitution(
        [FindPackageShare("wheel_leg_mujoco"), "simulation", "model", "scence.xml"]
    )
    model_path = LaunchConfiguration("model_path")
    config_file = LaunchConfiguration("config_file")
    floating_base = LaunchConfiguration("floating_base")
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wheel_leg_mujoco"), "config", "fixed.yaml"]
                ),
            ),
            DeclareLaunchArgument("floating_base", default_value="false"),
            Node(
                package="wheel_leg_ros",
                executable="controller_node",
                name="wheel_leg_controller",
                output="screen",
            ),
            Node(
                package="wheel_leg_mujoco",
                executable="mujoco_node",
                name="wheel_leg_mujoco",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "model_path": model_path,
                        "floating_base": floating_base,
                    }
                ],
            ),
        ]
    )
