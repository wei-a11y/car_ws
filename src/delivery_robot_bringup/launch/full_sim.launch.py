from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, arguments):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    manager_share = Path(get_package_share_directory("delivery_robot_task_manager"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_yaw = LaunchConfiguration("yaw")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    sites_file = LaunchConfiguration("sites_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            # Gazebo world pose and AMCL map pose are distinct coordinate sets.
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_x", default_value="-14.65"),
            DeclareLaunchArgument("initial_y", default_value="-4.375"),
            DeclareLaunchArgument("initial_yaw", default_value="1.2272"),
            DeclareLaunchArgument(
                "sites_file",
                default_value=str(manager_share / "config" / "mission_sites.yaml"),
            ),
            include(
                "car_navigation",
                "sim_navigation.launch.py",
                {
                    "gui": gui,
                    "rviz": rviz,
                    "x": spawn_x,
                    "y": spawn_y,
                    "yaw": spawn_yaw,
                },
            ),
            include(
                "delivery_robot_module_mocks",
                "module_mocks.launch.py",
                {"use_sim_time": use_sim_time},
            ),
            include(
                "car_chassis_task",
                "chassis_task.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "initial_x": initial_x,
                    "initial_y": initial_y,
                    "initial_yaw": initial_yaw,
                    "require_module_safety": "true",
                },
            ),
            include(
                "delivery_robot_task_manager",
                "task_manager.launch.py",
                {"use_sim_time": use_sim_time, "sites_file": sites_file},
            ),
        ]
    )
