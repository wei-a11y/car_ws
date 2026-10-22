# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ============================================================
    # 1. 获取现有 ROS 2 包的安装目录
    # ============================================================

    share = Path(
        get_package_share_directory('car_description')
    )

    plan = Path(
        get_package_share_directory('plan')
    )

    controller = Path(
        get_package_share_directory('controller')
    )

    # 机器人 URDF
    urdf_file = share / 'urdf' / 'car_urdf.urdf'

    # 注意：必须确认这个文件确实存在
    robot_description = urdf_file.read_text(encoding='utf-8')

    # 导航参数
    config = LaunchConfiguration('navigation_params_file')

    # ============================================================
    # 2. 启动参数
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'start_sim',
            default_value='false',
            description='实体小车模式，不启动 Gazebo'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true'
        ),

        # 保留 navigation 原有地图路径，不使用 sim.launch.py 的地图路径
        DeclareLaunchArgument(
            'map_yaml',
            default_value=str(
                "/home/xinxinzi/cars_ws/car_ws/map/maps/room.yaml"
            ),
            description='Navigation map yaml'
        ),

        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=str(
                share / 'config' / 'amcl.yaml'
            )
        ),

        DeclareLaunchArgument(
            'navigation_params_file',
            default_value=str(
                share / 'config' / 'navigation_baseline.yaml'
            )
        ),

        DeclareLaunchArgument(
            'enable',
            default_value='true',
            description='Enable or disable Safety Gate'
        ),

        DeclareLaunchArgument(
            'controller_type',
            default_value='dwa',
            description='Controller selection: dwa or lqr'
        ),

        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity command output topic'
        ),

        # ========================================================
        # 3. 机器人模型：不启动 Gazebo
        # ========================================================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        robot_description,
                        value_type=str
                    ),
                    'use_sim_time': False,
                }
            ],
        ),

        # ========================================================
        # 4. RViz：实体车使用系统时间
        # ========================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='navigation_rviz',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('rviz')
            ),
            arguments=[
                '-d',
                str(share / 'config' / 'navigation.rviz')
            ],
            parameters=[
                {
                    'use_sim_time': False
                }
            ],
        ),
     Node(
          package='bringup',
          executable='odom2tf',
          name='odom2tf',
          output='screen',
          ),
        # ========================================================
        # 5. 地图服务器：使用 navigation 的 map_yaml
        # ========================================================

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {
                    'yaml_filename': LaunchConfiguration('map_yaml'),
                    'use_sim_time': False,
                }
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['map_server'],
                }
            ],
        ),

        # ========================================================
        # 6. AMCL 定位
        # ========================================================

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                LaunchConfiguration('amcl_params_file'),
                {
                    'use_sim_time': False
                }
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['amcl'],
                }
            ],
        ),

        # ========================================================
        # 7. 栅格地图处理
        # ========================================================

        Node(
            package='plan',
            executable='planning_grid_node',
            name='planning_grid',
            output='screen',
            parameters=[
                str(plan / 'config' / 'planning_grid.yaml'),
                config,
                {
                    'use_sim_time': False
                }
            ],
        ),

        # ========================================================
        # 8. A* 路径规划
        # ========================================================

        Node(
            package='plan',
            executable='astar_node',
            name='planner',
            output='screen',
            parameters=[
                str(plan / 'config' / 'astar.yaml'),
                config,
                {
                    'use_sim_time': False
                }
            ],
        ),

        # ========================================================
        # 9. DWA 控制器
        # ========================================================

        Node(
            package='controller',
            executable='dwa_tracker_node',
            name='dwa_controller',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    "'",
                    LaunchConfiguration('controller_type'),
                    "' == 'dwa'"
                ])
            ),
            parameters=[
                str(controller / 'config' / 'dwa_tracker.yaml'),
                config,
                {
                    'use_sim_time': False
                }
            ],
        ),

        # ========================================================
        # 10. LQR 控制器：需要时才启动
        # ========================================================

        Node(
            package='controller',
            executable='lqr_tracker_node',
            name='lqr_controller',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    "'",
                    LaunchConfiguration('controller_type'),
                    "' == 'lqr'"
                ])
            ),
            parameters=[
                str(controller / 'config' / 'lqr_tracker.yaml'),
                config,
                {
                    'use_sim_time': False
                }
            ],
        ),

        # ========================================================
        # 11. Safety Gate
        # ========================================================

        Node(
            package='controller',
            executable='safety_gate_node',
            name='safety_gate',
            output='screen',
            parameters=[
                str(controller / 'config' / 'safety_gate.yaml'),
                {
                    'use_sim_time': False,
                    'enable': ParameterValue(
                        LaunchConfiguration('enable'),
                        value_type=bool
                    ),
                    'cmd_vel_topic': ParameterValue(
                        LaunchConfiguration('cmd_vel_topic'),
                        value_type=str
                    ),
                }
            ],
        ),
    ])
    