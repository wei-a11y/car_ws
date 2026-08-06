from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = Path(get_package_share_directory("car_description"))
    navigation_share = Path(get_package_share_directory("car_navigation"))

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_yaw = LaunchConfiguration("yaw")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(description_share / "launch" / "sim.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "rviz_child": "false",
            "world": world,
            "x": spawn_x,
            "y": spawn_y,
            "yaw": spawn_yaw,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "navigation.launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": "true",
            "autostart": autostart,
            "use_composition": use_composition,
            "rviz": rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=str(description_share / "worlds" / "navigation.world"),
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map",
                default_value=str(navigation_share / "maps" / "edited.yaml"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(navigation_share / "config" / "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            simulation,
            navigation,
        ]
    )
