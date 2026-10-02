# Phase 6：低速 LQR Path Tracker

## 范围与部署前置条件

实现位于 `src/controller`：独立 C++ `lqr_tracker_core`、ROS `lqr_tracker_node`、
YAML、launch、核心及 ROS 测试。没有修改 URDF/Gazebo、Planner、Task 或共享接口配置。
没有 MPC、jerk、换挡、硬件 brake hold、深度相机或第三方控制库。

**输出仅为 `/cmd_vel_raw`，不是 `/cmd_vel`。Safety Gate 尚未实现，本阶段不能安全地
启动完整自主导航，也不能把 raw command remap 到底盘来绕过 Safety Gate。**
节点拒绝输出到已确认的 `/cmd_vel` 或 `/diff_drive_controller/cmd_vel_unstamped`。
实际规划 frame/localization 尚未确认；必须有真实的 Path frame -> base_footprint TF 链，
不新增假 map -> odom TF，不从 Task 读取 start pose。

## 接口

| 输入/输出 | 默认接口 | 类型 / 语义 |
| --- | --- | --- |
| Processed path | `/plan` | `nav_msgs/msg/Path`，reliable/volatile/depth 1 |
| Current pose | 显式 `planning_frame -> base_footprint` | 最新 TF，不默认回退 odom pose |
| Actual velocity | `/odom` | `nav_msgs/msg/Odometry`，best_effort/volatile/depth 5 |
| Raw command | `/cmd_vel_raw` | `geometry_msgs/msg/Twist`，reliable/volatile/depth 1 |
| Status | `/controller/lqr/status` | `std_msgs/msg/String`，reliable/transient_local/depth 1 |
| Debug | `/controller/lqr/debug` | `std_msgs/msg/Float64MultiArray`，reliable/volatile/depth 1 |

Odometry 必须是 `header.frame_id=odom`、`child_frame_id=base_footprint`；这些 frame
来自参数，必须与实际部署一致。只读取 body-frame 的 `linear.x/angular.z` 确认速度与停车，
不会把 odometry 的 pose 当作 planning-frame pose。
所有 topic 可通过参数/remap 设置，但 remap 后必须互不重名。

Path 每个 pose 必须具有 matching frame、有限坐标、单位 quaternion。
控制方向重新由 XY 线段几何计算，不把 Path stamp 当作速度轨迹或心跳。
Path 消费方需先启动，再发目标：当前 Path QoS 不为晚加入的节点保留旧路径。

## 数学模型及几何处理

状态为横向误差 `e_y`（机器人位于线段左侧为正）和
`e_theta=wrap(robot_yaw-segment_yaw)`，角度归一化到 `[-pi,pi)`。
直线段上 `e_y_dot≈v0*e_theta`、`e_theta_dot=w`；正标称速度 v0、周期 T 的 ZOH 模型：

```text
A = [[1, v0*T], [0, 1]]
B = [[0.5*v0*T*T], [T]]
Q = diag(q_lateral, q_heading), R = r_angular
w = clamp(-K[0]*e_y-K[1]*e_theta, -w_max, w_max)
```

启动时用 2x2 离散 Riccati 迭代计算固定 K，检查收敛、有限值及直段离散稳定性条件。
默认 T=0.05 s、v0=0.15 m/s、Q=diag(100,25)、R=4，K 约为 `[4.664,2.615]`。
K 不硬编码、不在 v=0 求解；正速度降低时的局部闭环稳定性已检查，但不意味着
饱和、快速变速、侧滑或定位跳变下仍有全局稳定保证。

- 将连续重复点和浮点 roundoff 范围内同方向共线点合并，保留真实拐点/折返。
- 当前直段上做有界线段投影 nearest；进度由实际 TF 更新，不积分已发送的 command。
  不跨过尚未停车的拐点去选远处最近点，避免自交路径跳段。
- lookahead 按米计算，限制在当前直段内，用于参考预览/调试；误差仍使用 nearest，
  不将 LQR 偷换成追逐 lookahead 点的控制器。
- `v_ref=min(v_nominal,slowdown_gain*remaining)`；remaining 是到下一个必须停车点的距离。
  `v` 限制在 `[0,v_max]`，没有强制爬行最低速度；`w` 限制在 `[-w_max,w_max]`。
- 每个真实拐点先输出零，等待实际速度达停车阈值，再原地对齐下一直段，最后恢复前进。
  大航向误差也先停再对齐。`BRAKING` 状态仅表示零 Twist 等待 Odometry，非硬件制动逻辑。
- 终点按实际 XY 距离及实测线/角速度判定到达；GOAL_REACHED 锁存并持续发布零，
  直到新 Path。索引到末端不等于到达；超出捕获窗口、跟踪误差超限或越过未到达的
  端点会锁存 TRACKING_ERROR，需要新 Path，不会盲目继续前进。
- 单点 Path 只能在实际位置已达容差时停车完成，否则拒绝跟踪；不凭空生成连接段。
  最终目标是 Path 末端 cell 中心，不是原始 Goal 精确位置/朝向；不实现终点强制朝向。

## 数据失效与零输出

无/空/非法 Path、新 Path 替换、Path publisher 消失都会清除旧命令。Publisher 消失后
需要重新收到有效 Path；DDS graph 的消失检测不是独立安全认证机制。
Path 不按创建时间过期，不能照抄底盘 0.5 s 的命令超时作为 Path 寿命。

缺失/无效/过期 TF 或 Odometry、错误 Odometry frame、无效/跳变控制时间都输出零。
这类外部数据故障恢复后会重新检查 TF/进度再继续；核心 TRACKING_ERROR 不自动恢复。
时间戳为零的静态 TF 允许用于隔离的合成测试，但不代表真实 localization。

控制 timer 使用 ROS clock，符合 Gazebo 仿真时间；额外 wall watchdog 在 ROS clock
暂停/缺失时持续输出零。watchdog 只在本节点仍运行且 executor 可执行时有效。
节点崩溃/被杀不能保证最后一条零命令送达，需要独立 Safety Gate 的 raw-command
超时和底盘现有 `cmd_vel_timeout`；本阶段没有实现 Safety Gate。

## Build / Test

在仓库根目录，所有输出留在仓库内：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/phase6_lqr build --base-paths src \
  --build-base build/phase6_lqr --install-base install/phase6_lqr \
  --packages-select controller plan
source install/phase6_lqr/local_setup.bash
mkdir -p log/phase6_lqr/ros
export ROS_LOG_DIR="$PWD/log/phase6_lqr/ros"
colcon --log-base log/phase6_lqr test \
  --build-base build/phase6_lqr --install-base install/phase6_lqr \
  --packages-select controller plan
colcon test-result --test-result-base build/phase6_lqr --verbose
```

测试包括数学矩阵/DARE/权重/反馈符号、wrap、nearest/lookahead、限幅、空/单点 Path、
拐点停车与对齐、终点持续零、理想非线性差速模型跟踪、失效/过期/frame 错误归零、
仿真时钟暂停、默认禁止 odom pose fallback 和禁止直接输出底盘。ROS 测试隔离 domain，
不会发送底盘速度命令。plan 的已有 Phase 2/3/4 测试同时回归。
XML lint 需访问 ROS 官方 schema；cppcheck 2.7 可能被 ament 自动跳过，需检查最终报告。

## 运行及调试

每个终端 source 上述环境，并使用相同的 ROS domain、sim-time 设置。
先只验证 raw 输出，不连接底盘。填写实际确认的规划 frame：

```bash
ros2 launch controller lqr_tracker.launch.py planning_frame:="实际已确认的Path frame"
ros2 topic echo /cmd_vel_raw
ros2 topic echo /controller/lqr/status --qos-durability transient_local
ros2 topic echo /controller/lqr/debug
```

之后启动已有 Planner/grid，在 RViz 发目标。TF 或 Odometry 不具备时，零输出和相应
UNAVAILABLE 状态是预期结果，不能用假 TF 补出实际定位再尝试自主运动。

Debug array 的固定字段顺序：

```text
e_y[m], e_theta[rad], progress[m], remaining[m], v_ref[m/s], v_cmd[m/s], w_cmd[rad/s],
nearest_x[m], nearest_y[m], lookahead_x[m], lookahead_y[m]
```

DEBUG 日志每秒打印误差、进度、速度和预览点；状态变化与启动 K 使用 INFO 日志。
可用下面方式覆盖参数并开启 debug（停止原节点后再启动，不启动两个 raw publisher）：

```bash
ros2 run controller lqr_tracker_node --ros-args \
  --params-file src/controller/config/lqr_tracker.yaml \
  -p planning_frame:="实际已确认的Path frame" \
  -p v_nominal:=0.1 -p v_max:=0.1 -p r_angular:=8.0 \
  --log-level lqr_controller:=debug
```

无 Gazebo 也可运行上面的 ROS 测试检查接口，测试 TF/odom 不作为实际机器人部署配置。
完整 Gazebo 自主闭环与实车调试必须等待真实 map/localization 和 Safety Gate 就绪。

## 调参顺序与限制

参数在 `src/controller/config/lqr_tracker.yaml`；全部为只读启动参数，修改 YAML/启动
override 后重启，不能用 `ros2 param set` 静默热改 K。

1. 先确认 Path/TF/Odometry frame、时间、Twist 符号和命令唯一发布方，再调 Q/R。
2. 从较低的正 v_nominal 开始。v_max/w_max 不超过现有底盘配置 0.5 m/s、1.0 rad/s；
   这些只是当前 Gazebo 配置，不能冒充实车已测限制。
3. 横向回归慢可逐步增大 q_lateral；转向过强/频繁饱和可增大 r_angular。
   q_heading 控制航向误差权重；每次只改变一组值，观察符号、振荡和饱和比例。
4. slowdown_gain 控制接近停止点的速度下降；align_gain/heading_tolerance 控制原地对齐。
   不使用最低速度 floor 强行穿越终点。
5. lookahead 只调预览范围，不是隐藏的 Pure Pursuit 转向增益；nearest window 必须
   覆盖合理的控制周期位移，又不能放宽到允许定位跳变跳过停点。
6. 调整 goal_position_tolerance、停车速度阈值需结合 Odometry 噪声和底盘死区。
   max_tracking_error 默认 0.03 m，仍需结合 Phase 2 的 0.05 m safety margin 与实际
   定位误差验证；不能为消除故障直接任意放大误差预算。

A* 首点是 cell 中心，真实 TF 与该点的偏差可能超出初始捕获预算，尤其地图较粗时。
应核对分辨率、定位和误差预算，不追加未经 inflated-grid 碰撞验证的捕获连接段。
碰撞安全的 Path 不等于实际跟踪轨迹一定安全；本 tracker 不替代 Safety Gate 或障碍检测。
