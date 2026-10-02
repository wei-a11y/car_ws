# Phase 8：单目标 Gazebo 闭环

## 范围与前置条件

复用现有 Grid/Inflation、A*、Path Processing、LQR 和 Safety Gate；独立使用
`nav2_amcl` 定位，不启动 Nav2 planner/controller/BT/costmap。没有修改算法核心、
URDF、世界/地图资产、Task，也没有添加深度相机。

新入口是 `ros2 launch car_description navigation.launch.py`。它包含已有仿真/地图
入口，再启动 AMCL、独立定位 lifecycle manager 和四个算法/安全节点。
`start_sim:=false` 只启动定位/导航，要求已有同 domain 的仿真和 map server，不能
同时启动两套地图、定位或控制节点。Gazebo 环境与地图默认使用已集成资产：

- `world_model/model.world`
- `map/edited/edited.yaml`

**文件对应关系不等于坐标对齐已验收。**当前机器人仍在 Gazebo `(0,0)` 生成；
地图 origin 为 `[-23.3,-16.9,0]`，不能据此推导机器人地图位姿或 `map -> odom`。
必须从场景和地图确认初始位置/朝向，用 RViz 初始化，并观察激光点与墙面是否吻合。
对齐不成立、地图不匹配或 TF 不完整时，不发送自主导航目标；先报告前置问题。

## 接口与参数

```text
Gazebo -> /scan + /odom + odom -> base_footprint
URDF/RSP -> base_footprint -> base_link -> laser_link
/map + /scan + odometry TF + /initialpose -> AMCL -> map -> odom
/map -> grid/inflation -> /plan/inflated_grid
/goal_pose + map -> base_footprint TF -> A* / post-processing -> /plan
/plan + TF + /odom -> LQR -> /cmd_vel_raw
/cmd_vel_raw + /scan + sensor TF -> Safety Gate -> /cmd_vel -> Gazebo
```

- AMCL 使用 `config/amcl.yaml`；由 lifecycle manager configure/activate。
  `set_initial_pose=false`，不写未经确认的默认初始位姿，不创建静态 `map -> odom`。
  AMCL 是 `map -> odom` 的唯一发布者；底盘仍发布 `odom -> base_footprint`。
- `config/navigation_baseline.yaml` 是部署覆盖，不替换 Phase 2～7 默认文件。
  planning frame 为 `map`；`robot_radius=0.355 m` 沿用已确认值，安全余量提高到
  `0.10 m`，LQR `max_tracking_error=0.04 m` 容纳 5 cm cell 中心的起点量化。
  这些是低速调试值，不是实测定位精度，也不证明安全；定位/跟踪/栅格误差总预算
  必须小于余量。误差超限应排查定位/对齐，不能不断放宽阈值。
- 同一个覆盖文件设置 Gazebo `publish_rate=100.0`，使 `/clock` 快于 LQR 20 Hz。
  默认 Gazebo 10 Hz 时钟会造成重复控制回调在同一 ROS 时间，触发现有零输出保护。
  若使用 `start_sim:=false`，已有仿真也必须应用这个 Gazebo 参数。
- 自动模式只允许 Safety Gate 发布底盘 `/cmd_vel`。退出 keyboard teleop，
  不将 `/cmd_vel_raw` remap 到底盘。不加入 mux 或旁路。
- `enable:=false` 仍持续零输出，**不是绕过安全检查**。参数修改后重新启动。
- `/plan` 是 volatile；先启动 LQR/RViz，再发目标。地图更新会清空旧路径，需要
  重新发目标；没有自动重规划、动态绕障或多目标 Task。
- 到达语义是 Path 末端 cell 中心的 XY 容差及 `/odom` 实际停车；不保证原始 Goal
  精确位置或最终朝向。规划 `SUCCESS` 不是到达，控制 `GOAL_REACHED` 才表示完成。

## Build 与自动测试

在仓库根目录，输出保持在仓库内：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/phase8_navigation build --base-paths src \
  --build-base build/phase8_navigation --install-base install/phase8_navigation \
  --packages-select car_description plan controller
source install/phase8_navigation/local_setup.bash
mkdir -p log/phase8_navigation/ros
export ROS_LOG_DIR="$PWD/log/phase8_navigation/ros"
colcon --log-base log/phase8_navigation test \
  --build-base build/phase8_navigation --install-base install/phase8_navigation \
  --packages-select plan controller
colcon test-result --test-result-base build/phase8_navigation --verbose
python3 -m pytest -q src/urdf/test/test_navigation.py \
  --basetemp=log/phase8_navigation/pytest -o cache_dir=log/phase8_navigation/pytest_cache \
  --junitxml=log/phase8_navigation/integration.xml
ament_flake8 src/urdf/launch src/urdf/test tools/phase8_metrics.py
xmllint --noout src/urdf/package.xml
git diff --check
```

新增 pytest 需 ROS 环境、pytest、PyYAML、rosgraph_msgs、TF 和 AMCL。
它使用独立 localhost domain 226～230、合成房间/激光/里程计/时钟和理想差速运动学；
速度输出只到 `/phase8_test/cmd_vel`，不启动 Gazebo，不发布底盘 `/cmd_vel`。
覆盖真实 AMCL 初始化与动态 TF、Grid→A*→LQR→Gate 单目标到达/持续停车、
INVALID_GOAL/NO_PATH 清除路径、障碍停止/释放、Scan/TF/时钟失效及 enable=false。
**这不是实际 Gazebo 场景、地图配准或八类场景的闭环验收。**

## 启动与 RViz 验证

所有终端 source 同一环境并设置相同 ROS domain，不运行旧的仿真/导航进程。

```bash
ros2 launch car_description navigation.launch.py
```

如需覆盖资产或不显示 GUI：

```bash
ros2 launch car_description navigation.launch.py \
  world:="$PWD/world_model/model.world" map_yaml:="$PWD/map/edited/edited.yaml"
ros2 launch car_description navigation.launch.py gui:=false rviz:=false
```

以上命令是不同启动方式，**不要同时执行**。`sim.launch.py` 仍可单独用于遥控，
默认 RViz 配置不变；闭环入口传入新的 `config/navigation.rviz`，Fixed Frame 为 map。

启动后依次：

1. 确认 map server 和 AMCL active，底盘控制器 active，`/clock`、`/scan`、`/odom` 有数据。
2. RViz 点 `2D Pose Estimate`，在地图中设置实际初始位置/朝向；检查激光与墙面对齐。
3. 检查 `map -> base_footprint` 持续可用，机器人落在 inflated grid 可通行区域。
4. 确认 `/cmd_vel` 唯一发布者为 Safety Gate。RViz 已订阅 Path，LQR 已就绪。
5. 在空旷区域选择短直线目标，用 `2D Goal Pose` 发布 `/goal_pose`；观察实际车体、
   路径与状态，最终 GOAL_REACHED、raw/final Twist 都为零、Odometry 速度接近零。

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 control list_controllers
ros2 param get /gazebo publish_rate
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic info /cmd_vel --verbose
ros2 topic echo /plan/status --qos-durability transient_local
ros2 topic echo /controller/lqr/status --qos-durability transient_local
ros2 topic echo /controller/safety/status --qos-durability transient_local
```

前置条件未完成前，不把仅有 TF 连通当作定位可靠。不要人为制造假定位让底盘运动。
Safety Gate 仅检查前向扇区，原地转向/侧后方不受完整保护；首次调试必须在车体
全周有充足净空的位置。深度相机、DWA/TEB 和局部绕障不在本阶段范围。

## 场景验收与指标

| 场景 | 验收条件 |
| --- | --- |
| 短直线 | 无异常抖动，到达并持续停车 |
| 90° 转弯 | 拐点停车/对齐后继续，实际车体无碰撞 |
| 多拐点 | 不跳段、不提前到达，逐段完成 |
| 静态障碍绕行 | A* 路径绕过 inflated cells，实际车体无碰撞 |
| 窄通道 | 净空足够才通过，否则拒绝规划；不为通过而缩小确认的机器人尺寸 |
| invalid goal | INVALID_GOAL、空 Path，停止旧任务 |
| NO_PATH | 合法起终点不连通，NO_PATH、空 Path，不盲目前进 |
| LiDAR stop | 近障停止，迟滞释放后按原路径恢复；不期望动态重规划 |

另外验证停止 Scan、停止 LQR、定位/TF 失效和暂停 Gazebo 都停止自动运动。
Safety stop 不等于 navigation success；恢复后需要重新检查定位和路径有效性。

发送目标前启动记录，每次使用新的输出目录：

```bash
ros2 bag record --use-sim-time -o log/phase8_navigation/run_01 \
  /clock /map /plan/inflated_grid /goal_pose /plan/raw_path /plan /plan/status \
  /controller/lqr/status /controller/lqr/debug /controller/safety/status \
  /cmd_vel_raw /cmd_vel /odom /scan /tf /tf_static /amcl_pose
# 停止记录后，离线计算；脚本只读 bag，不发任何 ROS 消息。
python3 tools/phase8_metrics.py log/phase8_navigation/run_01
```

离线 JSON 提供 goal→planning status 的观察延迟（**不是 A* CPU 时间**）、raw/processed
路径长度、TRACKING 时的横向 RMSE/最大误差、heading RMSE、末端位置误差、导航时间
和结果。TF 末端误差使用最近接收的 map→odom/odom→base 估计，无时间插值，非 Gazebo
ground truth；未定位/未记录所需 TF 则为 null。未完成的 bag 标记 INCOMPLETE，不算成功。
每个场景需记录实际车体是否碰撞；定位估计指标不能替代物理碰撞验收。

## 本次实际验证范围

- 三个包构建成功；新增 pytest 5 项通过，包含指标计算测试。
- plan/controller 回归测试通过；XML lint 首次受沙箱内 schema 访问限制，重跑成功。
  `colcon test-result` 为 0 errors / 0 failures / 19 skipped（cppcheck 版本相关跳过）。
- Python lint、XML 语法、launch 参数展开、git diff whitespace 检查通过。
- 离线指标脚本通过真实 rosbag2 写入/读取 round-trip，未完成目标不标记成功。
- 独立 domain 231 / Gazebo master 11355，关闭 GUI、禁用自动运动、输出仅到调试
  topic，确认世界和机器人加载、地图发布、控制器启动、Scan/Odom 发布、AMCL active。
  5 秒内收到 499 条 clock、50 条 scan、250 条 odom；Gazebo publish_rate 为 100 Hz，
  LQR 为 WAIT_PATH。未初始化前无 map→odom，调试速度为零，无 `/cmd_vel` 发布者。
- **未完成**实际地图/场景初始位姿配准、RViz GUI 交互及八类 Gazebo 自主运动验收。
  不将合成运动学测试当作真实 Gazebo 到达/碰撞测试。运行时需先完成定位对齐，再发目标。
