# car_navigation

本包提供当前差速底盘的 Nav2 静态地图定位和仿真点到点导航。

## 构建

```bash
cd ~/my/ws/car_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to car_navigation
source install/setup.bash
```

如果 `rosdep` 提示尚未初始化，先执行 `sudo rosdep init` 和 `rosdep update`；
当前工作区所需的 Nav2 依赖已经安装，可直接执行 `colcon build`。

当前环境已经安装 ROS 2 Humble 的 Navigation2、Nav2 Bringup、AMCL 和
Map Server；新环境至少需要安装：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 一键启动仿真导航

```bash
ros2 launch car_navigation sim_navigation.launch.py
```

无图形界面运行：

```bash
ros2 launch car_navigation sim_navigation.launch.py gui:=false rviz:=false
```

只启动 Nav2、AMCL、地图和 RViz，用于连接已经启动的底盘仿真：

```bash
ros2 launch car_navigation navigation.launch.py
```

可以通过 launch 参数替换地图、参数、世界和出生位姿：

```bash
ros2 launch car_navigation sim_navigation.launch.py \
  map:=/absolute/path/map.yaml \
  x:=0.0 y:=0.0 yaw:=0.0
```

## RViz 操作

1. 等待地图、机器人和激光出现。
2. 点击 `2D Pose Estimate`，在地图中设置机器人当前位置和朝向。
3. 确认 AMCL 粒子收敛、TF 中出现 `map -> odom`，两个代价地图完成激活。
4. 点击 `Nav2 Goal` 设置导航目标。
5. 观察 `/plan`、`/local_plan`、机器人 footprint 和 `/cmd_vel`。

首次定位完成后，将实际测得的初始位姿和两个自由区域目标写入
`config/site_waypoints.yaml`，并把 `calibrated` 改成 `true`。在此之前该文件
中的零值只是明确禁用的占位值，不会被启动文件自动发布。

## 接口和 TF 所有权

| 功能 | 接口/发布者 |
|---|---|
| 激光 | `/scan` (`sensor_msgs/msg/LaserScan`) |
| 里程计 | `/odom` (`nav_msgs/msg/Odometry`) |
| 速度 | `/cmd_vel` (`geometry_msgs/msg/Twist`) |
| 导航 | `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) |
| `map -> odom` | AMCL |
| `odom -> base_footprint` | `diff_drive_controller` |
| 底盘和传感器静态/关节 TF | `robot_state_publisher` |

不要在导航运行期间同时启动键盘遥控节点，否则它会与 Nav2 竞争速度控制权。
Nav2 的恢复行为会按官方 Humble 启动结构在执行恢复时直接发布 `/cmd_vel`；
正常路径跟踪命令经过 `/cmd_vel_nav` 和 `velocity_smoother` 后到达 `/cmd_vel`。

## 启动后检查

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo map base_footprint
```

所有 lifecycle 节点应为 `active`。`/scan` 的 `frame_id` 应为 `laser_link`，
TF 应为 `map -> odom -> base_footprint -> base_link -> laser_link`。

## 首期验收

- 在地图中选择至少三个方向和距离不同的自由目标。
- 连续执行十次导航，至少九次成功。
- 最终位置误差不超过 0.10 m，航向误差不超过 0.087 rad。
- 障碍内部或地图外目标应规划失败，底盘保持停止。
- 取消执行中的目标后，底盘应及时停止且 Action 返回取消状态。

本阶段不包含 SLAM、实车驱动、双雷达融合、IMU 融合和精确对接。
