from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = Path(get_package_share_directory("car_navigation"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    rviz_enabled = LaunchConfiguration("rviz")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / "launch" / "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_rviz",
        output="screen",
        arguments=["-d", str(navigation_share / "rviz" / "navigation.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=str(navigation_share / "maps" / "edited.yaml"),
                description="Absolute path to a map_server YAML file.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(navigation_share / "config" / "nav2_params.yaml"),
                description="Nav2 parameter file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("rviz", default_value="true"),
            nav2,
            rviz,
        ]
    )
