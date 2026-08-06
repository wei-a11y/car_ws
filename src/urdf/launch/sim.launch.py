import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("car_description"))
    urdf_file = package_share / "urdf" / "urdf_1.urdf"
    robot_description = urdf_file.read_text(encoding="utf-8")
    robot_description = robot_description.replace(
        "$(find car_description)", str(package_share)
    )
    # gazebo_ros2_control passes the URDF through the ROS argument parser.
    # Absolute mesh filenames avoid parser issues with URI colons and remain
    # valid because this launch file already resolved the package share path.
    robot_description = robot_description.replace(
        "package://car_description/",
        f"file://{package_share}/"
    )

    gazebo_launch = Path(get_package_share_directory("gazebo_ros")) / "launch" / "gazebo.launch.py"

    gui = LaunchConfiguration("gui")
    rviz_enabled = LaunchConfiguration("rviz_child")
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_yaw = LaunchConfiguration("yaw")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            "gui": gui,
            "server": "true",
            "verbose": "false",
            "world": world,
        }.items(),
    )

    # Gazebo resolves package://car_description/... meshes as
    # model://car_description/... and searches the parent of the package
    # share directory in GAZEBO_MODEL_PATH.
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            str(package_share / "models"),
            os.pathsep,
            str(package_share.parent),
            os.pathsep,
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_car",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "car",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.05",
            "-Y", spawn_yaw,
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(package_share / "config" / "car.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz_enabled),
    )

    load_diff_drive_after_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_controller],
        )
    )

    load_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Set to false to run Gazebo headless.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Set to false to skip RViz.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=str(package_share / "worlds" / "navigation.world"),
            description="Gazebo world file.",
        ),
        DeclareLaunchArgument("x", default_value="0.0", description="Robot spawn x."),
        DeclareLaunchArgument("y", default_value="0.0", description="Robot spawn y."),
        DeclareLaunchArgument(
            "yaw", default_value="0.0", description="Robot spawn yaw in radians."
        ),
        gazebo_model_path,
        gazebo,
        rviz,
        robot_state_publisher,
        spawn_entity,
        load_controllers_after_spawn,
        load_diff_drive_after_joint_states,
    ])
