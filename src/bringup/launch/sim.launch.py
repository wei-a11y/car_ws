# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _after_success(actions):
    def callback(event, context):
        if event.returncode != 0:
            reason = f'Simulation prerequisite exited with code {event.returncode}'
            return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
        return actions
    return callback


def _launch_simulation(context):
    package_share = Path(get_package_share_directory("car_description"))
    urdf_file = package_share / "urdf" / "car_urdf.urdf"
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
    with_arm = IfCondition(LaunchConfiguration('with_arm')).evaluate(context)
    world = LaunchConfiguration('world')
    arm_startup_handlers = []
    if with_arm:
        from bringup.description import gazebo_moveit_config
        from bringup.world import gazebo_world_file
        robot_description = gazebo_moveit_config(
            LaunchConfiguration('arm_params_file').perform(context)
        ).robot_description['robot_description']
        world_file = gazebo_world_file(
            world.perform(context), LaunchConfiguration('physics_params_file').perform(context))
        world = str(world_file)

        def cleanup_world(context):
            world_file.unlink(missing_ok=True)
            return []

        arm_startup_handlers.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=cleanup_world)])))

    gazebo_launch = (Path(get_package_share_directory("gazebo_ros")) /
                     "launch" / "gazebo.launch.py")

    gui = LaunchConfiguration("gui")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            "world": world,
            "gui": gui,
            "server": "true",
            "verbose": "false",
            "params_file": LaunchConfiguration("gazebo_params_file"),
        }.items(),
    )

    # Gazebo resolves package://car_description/... meshes as
    # model://car_description/... and searches the parent of the package
    # share directory in GAZEBO_MODEL_PATH.
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[f"{package_share.parent}:{package_share}:",
               EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')],
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
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
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

    combined_controllers = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['arm_controller', 'gripper_controller', 'arm_base_controller',
                   'joint_state_broadcaster', 'diff_drive_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
    ) if with_arm else None
    if with_arm:
        arm_startup_handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=combined_controllers, on_exit=_after_success([]))))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
    )

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[{
            "use_sim_time": True,
            "yaml_filename": LaunchConfiguration("map_yaml"),
        }],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen", parameters=[{
            "use_sim_time": True, "autostart": True, "node_names": ["map_server"],
        }],
    )

    load_diff_drive_after_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=_after_success([diff_drive_controller]),
        )
    )

    load_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            # Claim arm position interfaces first, before publishing or loading
            # the base controller. One spawner avoids competing service calls.
            on_exit=_after_success([combined_controllers] if with_arm else [
                joint_state_broadcaster]),
        )
    )

    return [
        *arm_startup_handlers,
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        load_controllers_after_spawn,
        load_diff_drive_after_joint_states,
        spawn_entity,
        rviz,
        map_server,
        map_lifecycle,
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory('car_description'))
    bringup_share = Path(get_package_share_directory('bringup'))
    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Set to false to run Gazebo headless.",
        ),
        DeclareLaunchArgument(
            "world", default_value=str(package_share / "world_model" / "model.world"),
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_params_file", default_value=""),
        DeclareLaunchArgument(
            "rviz_config", default_value=str(package_share / "config" / "car.rviz"),
        ),
        DeclareLaunchArgument(
            "map_yaml", default_value=str(package_share / "map" / "edited" / "edited.yaml"),
        ),
        DeclareLaunchArgument(
            'with_arm', default_value='false',
            description='Enable Gazebo arm/gripper trajectory controllers'),
        DeclareLaunchArgument(
            'arm_params_file', default_value=str(bringup_share / 'config/gazebo_arm.yaml'),
            description='Simulation-only joint limits, PID gains and initial positions'),
        DeclareLaunchArgument(
            'physics_params_file',
            default_value=str(bringup_share / 'config/gazebo_arm_physics.yaml'),
            description='ODE physics settings used only when with_arm is true'),
        OpaqueFunction(function=_launch_simulation),
    ])
