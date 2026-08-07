from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("delivery_robot_module_mocks"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="delivery_robot_module_mocks",
                executable="module_mocks",
                output="screen",
                parameters=[
                    share / "config" / "mocks.yaml",
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
        ]
    )
