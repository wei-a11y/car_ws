from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("car_chassis_task"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    require_module_safety = LaunchConfiguration("require_module_safety")

    common = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
    localization_overrides = {
        **common,
        "initial_x_m": ParameterValue(initial_x, value_type=float),
        "initial_y_m": ParameterValue(initial_y, value_type=float),
        "initial_yaw_rad": ParameterValue(initial_yaw, value_type=float),
    }
    chassis_overrides = {
        **common,
        "require_module_safety_states": ParameterValue(
            require_module_safety, value_type=bool
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("require_module_safety", default_value="true"),
            Node(
                package="car_chassis_task",
                executable="localization_supervisor",
                output="screen",
                parameters=[share / "config" / "localization.yaml", localization_overrides],
            ),
            Node(
                package="car_chassis_task",
                executable="velocity_arbiter",
                output="screen",
                parameters=[share / "config" / "chassis_task.yaml", common],
            ),
            Node(
                package="car_chassis_task",
                executable="chassis_task_server",
                output="screen",
                parameters=[
                    share / "config" / "chassis_task.yaml",
                    share / "config" / "side_alignment.yaml",
                    chassis_overrides,
                ],
            ),
        ]
    )
