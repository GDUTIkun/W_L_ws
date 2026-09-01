from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("wheel_leg_mujoco")
    model_path = LaunchConfiguration("model_path")
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [
                        package,
                        "simulation",
                        "model",
                        "scene_axisymmetric_centered_com_v1.xml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [package, "config", "current_weighted_wbc.yaml"]
                ),
            ),
            Node(
                package="wheel_leg_ros",
                executable="controller_node",
                name="wheel_leg_controller",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="wheel_leg_mujoco",
                executable="mujoco_node",
                name="wheel_leg_mujoco",
                output="screen",
                parameters=[config_file, {"model_path": model_path}],
            ),
        ]
    )
