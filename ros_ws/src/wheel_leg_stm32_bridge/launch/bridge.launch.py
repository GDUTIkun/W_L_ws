from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("wheel_leg_stm32_bridge"))
    default_config = str(package_share / "config" / "bridge.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("serial_device", default_value="/dev/ttyAMA4"),
            Node(
                package="wheel_leg_stm32_bridge",
                executable="stm32_bridge_node",
                name="stm32_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {"serial_device": LaunchConfiguration("serial_device")},
                ],
            ),
        ]
    )
