from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("delivery_robot_task_manager"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    sites_file = LaunchConfiguration("sites_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "sites_file",
                default_value=str(share / "config" / "mission_sites.yaml"),
            ),
            Node(
                package="delivery_robot_task_manager",
                executable="task_manager",
                output="screen",
                parameters=[
                    share / "config" / "task_manager.yaml",
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "sites_file": sites_file,
                    },
                ],
            ),
        ]
    )
