# bringup 启动入口

`bringup` 负责组合启动；`car_description`（源码目录 `src/urdf`）保存 URDF、mesh
和描述/导航配置；`arm_moveit_config` 保存 SRDF、运动学、控制器配置及 MoveIt
生成的组件启动文件。现有地图与世界源资产仍在仓库根目录，通过
`car_description` 的 package share 安装和查找。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to bringup
source install/setup.bash
```

## 入口

| 命令 | 用途 |
| --- | --- |
| `ros2 launch bringup display.launch.py` | RViz 模型与关节滑块，使用系统时间 |
| `ros2 launch bringup sim.launch.py` | 原 Gazebo 底盘/地图入口 |
| `ros2 launch bringup navigation.launch.py` | 原 AMCL + A* + LQR + Safety Gate 导航入口 |
| `ros2 launch bringup moveit_rviz.launch.py` | RViz MoveIt 规划/执行，使用 mock 硬件 |
| `ros2 launch bringup gazebo_moveit.launch.py` | Gazebo 底盘 + 机械臂/夹爪 + MoveIt |

上表是不同的启动方式。每个 ROS domain 中选择一套仿真/硬件启动入口。
Gazebo 与 RViz 可通过 `gui:=false`、`rviz:=false` 关闭；RViz mock 入口支持
`rviz:=false`。仿真入口支持 `world:=...`、`map_yaml:=...` 覆盖场景与地图。

## Gazebo 与 MoveIt

联合入口复用 Gazebo 创建的唯一 `controller_manager`，激活
`joint_state_broadcaster`、`diff_drive_controller`、`arm_controller`、
`gripper_controller`、`arm_base_controller`。它在启动时把现有 mock 描述的机械臂接口合并进
`GazeboSystem`，复用同一份模型供 Gazebo、RSP、MoveIt 和 RViz 使用；原始 URDF
保持独立底盘仿真所用配置。MoveIt/RViz 与控制器均使用仿真时间。

`arm` 仍只规划 `arm_1_joint`～`arm_5_joint`；平移关节 `arm_base_joint` 由
`arm_base_controller` 持位。末端拖动遵循当前五自由度 IK 设置，不能任意指定六维姿态。
Gazebo 机械臂和夹爪使用 `position_pid` 力/力矩控制，控制器读取真实位置；
`joint_state_broadcaster` 继续发布实际位置和速度。

仿真专用参数集中在 `config/gazebo_arm.yaml`，通过 `arm_params_file:=...` 覆盖，
包括初始姿态、速度/加速度、力矩/推力限制、PID 和关节物理阻尼。物理阻尼通过
URDF `dynamics/damping` 应用，并启用 ODE 隐式阻尼；它与 PID 的 `kd` 独立。
平移关节还配置 `dynamics/friction`，单位为 N；该值是关节摩擦力，
并非接触面的摩擦系数。`arm_base_controller` 持续保持最后的位置指令，
PID、阻尼和摩擦共同抑制底盘转向扰动；有限力矩的物理控制仍允许瞬态持位误差。
原 URDF 的几何和位置上下限保持；
这些参数尚未核对实机规格。Gazebo 的 JTC 使用位置状态初始化，确保首次持位目标来自
配置初值；控制器依次加载机械臂、夹爪、平移关节、状态广播和底盘。

联合入口还用 `config/gazebo_arm_physics.yaml` 生成临时世界文件，并在退出时清理。
对当前模型，ODE `world` 求解器配合轻微约束柔顺度解决了 `quick` 求解器下的高速越界。
世界场景、原世界文件和底盘单独仿真入口保持原配置；`physics_params_file:=...`
可以覆盖联合仿真的物理设置，`world:=...` 需提供单个 ODE physics 的 SDF 世界。
轨迹执行验收不等于物体接触抓取或实机力控验收。

联合入口默认只组合仿真与 MoveIt，不会启动 AMCL、A*、LQR 和 Safety Gate。
需要导航时，在另一个同 domain 的终端复用它。两个终端均 source 同一工作区，并设置
相同的 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`；隔离示例见下文：

```bash
ros2 launch bringup gazebo_moveit.launch.py
# 另一个终端，source 同一环境后执行：
ros2 launch bringup navigation.launch.py start_sim:=false
```

导航入口会打开独立的 `navigation_rviz`，加载
`car_description/config/navigation.rviz`，Fixed Frame 为 `map`。
在该窗口使用 **2D Pose Estimate** 设置有效 `/initialpose`；确认机器人位置、朝向和
激光扫描与地图对齐后，再用 **2D Goal Pose** 发布 `/goal_pose`。
本项目的 A* 消费 `/goal_pose`，没有完整 Nav2 的 `navigate_to_pose` action，
不要改用 **Nav2 Goal** 工具。MoveIt 窗口继续用于机械臂规划。
导航命令继续经过 Safety Gate；`rviz:=false` 可关闭导航窗口，
`start_sim:=false` 不会再启动 Gazebo、地图服务器或控制器。

导航完成后的抓取任务调度由后续任务节点实现。

## 机械臂与夹爪指令测试

`gazebo_moveit.launch.py` 和 `moveit_rviz.launch.py` 默认启动
`arm_command_bridge`；`arm_commands:=false` 可关闭该测试接口。
它复用已启动的 MoveIt，通过规划服务与轨迹执行 action 串行处理指令，
不启动新的机器人或控制器。也可单独运行：

```bash
ros2 launch bringup arm_command_bridge.launch.py
# 对于 RViz mock，使用 use_sim_time:=false；不要重复启动 bridge。
ros2 topic echo /arm/command_status
```

末端目标使用 `/arm/pose_goal` (`geometry_msgs/PoseStamped`)，
默认坐标系为 `base_footprint`，控制点为 `arm_5_link` 原点，并非额外定义的夹持 TCP。
五自由度机械臂默认只约束目标位置、忽略四元数；KDL 配置已启用 `position_only_ik`。
Gazebo 中需要姿态约束时，重新启动联合入口并设置 `position_only:=false`，
它会同时切换 KDL 和 bridge 的模式；规划仍需满足姿态约束，不能任意实现六维位姿。
单独启动 bridge 不会更改已有 MoveIt 的 IK 设置，需保证两者模式匹配。
参数在 `config/arm_command_bridge.yaml`。

下面的可达位置由现有模型 FK 求得，可在底盘、滑台静止且滑台为零时测试：

```bash
ros2 topic pub --once /arm/pose_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_footprint}, pose: {position: {x: 0.448266128, y: -0.010210671, z: 0.210513856}, orientation: {w: 1.0}}}"
```

该例使用默认位置模式。完整姿态模式先用 `position_only:=false` 重启联合入口，
再发送同一可达位置的对应姿态：

```bash
ros2 topic pub --once /arm/pose_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_footprint}, pose: {position: {x: 0.448266128, y: -0.010210671, z: 0.210513856}, orientation: {x: 0.049916708, y: -0.002497917, z: -0.049916708, w: 0.997502083}}}"
```

夹爪命令经过 MoveIt 规划、碰撞检查和执行，等待 `SUCCEEDED` 后再发下一条：

```bash
ros2 topic pub --once /arm/gripper_command std_msgs/msg/String "{data: open}"
ros2 topic pub --once /arm/gripper_command std_msgs/msg/String "{data: half}"
ros2 topic pub --once /arm/gripper_command std_msgs/msg/String "{data: close}"
```

对应两指位置分别为 `[-0.025, -0.025]`、`[-0.0125, -0.0125]` 和 `[0, 0]` m。
两个关节都使用负值张开；原关节范围是 `[-0.035, 0]` m。
执行中发送新命令会得到 `REJECTED_BUSY`；无效目标或规划/执行失败会明确报告。
若 `FAILED` 带 `busy_until_execution_stops: true`，执行停止尚未确认，接口仍忙；
等待后续终态并检查 MoveIt/controller 状态，不要把这条失败消息当作已经停止。
测试前先停止底盘和平移关节，确认 MoveIt 与关节反馈就绪。

平移关节不在 MoveIt `arm` 组中，独立位置指令仍为：

```bash
ros2 topic pub --times 10 --rate 5 --wait-matching-subscriptions 1 --keep-alive 1 \
  /arm_base_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.02]}"
```

位置单位是 m，沿 `base_link` 的 +Y 轴，范围 `[-0.12, 0.12]`。
`data: [0.0]` 回零。该位置话题不经过 MoveIt 规划，阶跃目标也没有轨迹速度规划；
测试时从小位移开始，MoveIt 执行期间保持平移关节不动。
该命令短时重复发送同一绝对位置，发送结束后控制器继续保持最后收到的目标。
实测中，订阅端为 `BEST_EFFORT` 时出现过 CLI `--once` 输出已发布但位置仍保持
旧目标的情况；上述重复发布能够回零。`publishing` 日志不是控制器已接收或执行的确认，
应以 `/joint_states` 中 `arm_base_joint` 的实际位置为准，发送后留出收敛时间。

## 检查

```bash
ros2 launch bringup gazebo_moveit.launch.py --show-args
ros2 control list_controllers
ros2 action list -t
ros2 topic echo /joint_states --once
```

应可看到机械臂与夹爪的 `follow_joint_trajectory` action。旧
`ros2 launch car_description ...` 入口已迁移到 `bringup`；如果原 install 目录残留
旧 launch 文件，使用新包入口或重新生成 install 目录。

## 本次验证结果

- `colcon build --packages-up-to bringup` 成功；五个已安装入口与源码一致且可加载。
- `bringup` 的 flake8、CMake lint、pep257、XML schema 检查通过。受限网络环境下
  XML 检查使用 `/tmp/car_ws_ros_schema/catalog.xml` 中映射的官方 ROS schema。
- 独立 ROS domain 78 中，RViz mock 入口的机械臂与夹爪均完成规划、执行和反馈验证。
- 加入物理阻尼后的独立 ROS domain 93、Gazebo 端口 11393 中，五个控制器均 active，唯一 Gazebo
  硬件系统、仿真时间、完整真实关节状态、里程计与 TF 检查通过。
  机械臂五关节和夹爪均完成规划、Execute 与反馈验收：最大终点误差分别约
  `0.000230 rad` 和 `0.000076 m`，全过程反馈位置/速度检查通过，平移关节及车轮保持静止。
  执行后继续观察 30 秒仿真时间（3001 条反馈），八个关节持续持位稳定；
  该测试没有发布 `/cmd_vel` 或 `/cmd_vel_raw`。
  启动仍观察到一次 ODE LCP 数值警告，后续未持续出现；其他负载/场景尚未验收。
- 滑轨摩擦/PID 修改在独立 domain 96/97 对比：±0.2 rad/s 平滑及阶跃转向和停止时，
  最大滑轨偏移从约 `0.806 mm` 降至 `0.458 mm`。0.02 m 平滑移动/回零和直接位置阶跃
  均完成；直接阶跃终点误差小于 0.1 mm，测得峰值速度约 0.166 m/s，
  此位置接口不提供轨迹速度规划。持位验收不代表任意底盘加速度下绝对零偏移。
- 末端位置话题在 domain 97 完成规划、执行和真实 FK 检查，终点位置误差约 `0.409 mm`；
  夹爪 open/half/close 均执行成功，最大实测指位置误差约 `0.140 mm`。
  domain 98 中，`position_only:=false` 的完整位姿目标也完成规划与执行，真实 FK
  位置误差约 `0.791 mm`，姿态误差约 `0.00247 rad`；三种夹爪命令再次全部成功。
  错误坐标系、非法夹爪命令以及执行中并发命令均被拒绝。MoveIt 的碰撞检查使用其当前
  planning scene；本入口不会自动把 Gazebo 世界物体导入该场景。
  本次测试 JSONL 保存在 `log/bringup_migration/validation/`，包括
  `slide_yaw_baseline.jsonl`、`slide_yaw_hold.jsonl`、`slide_step_commands.jsonl`、
  `arm_pose_commands.jsonl` 和 `arm_full_pose_commands.jsonl`。
- 原导航回归测试为 3 passed、2 failed：`max_tracking_error=0.2` 大于
  `safety_margin=0.1`；障碍测试使用 `0.4 m`，现停止距离为 `0.2 m`，未触发
  预期的 `OBSTACLE_STOP`。本次迁移保持了原导航参数与测试断言。

复查命令：

```bash
colcon test --packages-select bringup
colcon test-result --test-result-base build/bringup --verbose
python3 -m pytest -q src/bringup/test/test_navigation.py
```

机械臂执行回归在独立 ROS domain 的全新仿真中运行；两个终端设置相同 domain：

```bash
# 终端 1（source 同一环境）
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11391
ros2 launch bringup gazebo_moveit.launch.py gui:=false rviz:=false
# 终端 2（source 同一环境，并设置相同 ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY）
python3 src/bringup/tools/verify_moveit_execution.py --mode gazebo
```

该工具会检查初始持位、硬件、时钟、实际限位/速度与轮/滑轨漂移，然后规划并执行
机械臂和夹爪测试目标。RViz mock 回归使用新的 `moveit_rviz.launch.py rviz:=false`
并把工具模式改成 `--mode mock`。测试会移动仿真机械臂，须在隔离仿真中运行。

## 启动时出现控制器加载失败与时间回跳

`Failed loading controller joint_state_broadcaster` 会使联合入口关闭整套仿真；
后续 RViz/MoveIt 退出错误不一定是最初原因。若同时反复出现
`Detected jump back in time`，检查同一 ROS domain 是否存在其他 `/clock` 发布者
或同名 `/controller_manager`，以及当前 ROS 2 daemon 的缓存信息。
Gazebo 和 mock MoveIt 入口应分别使用独立 domain；局域网内其他 ROS 2 仿真也可能冲突。
本次排查用不依赖 ROS CLI daemon 的直接 DDS 图观察确认：domain 0 在允许网络通信时
发现了另一套 Gazebo、导航节点和控制器管理器；限制到 localhost 后，这些节点全部消失。
宿主机当时只有 domain 94 的验证仿真，说明观察到的另一套系统来自网络发现。

可在新的终端采用下面的隔离配置。已实际验证带 Gazebo GUI、RViz 和两个 MoveIt
轨迹执行的完整入口：五个控制器 active，期间没有时间回跳。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=94
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11394
ros2 launch bringup gazebo_moveit.launch.py
```

其他需要连接这套仿真的终端也设置相同 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY`。
`ROS_LOCALHOST_ONLY=1` 限制通信到本机；需要跨电脑通信时，应采用约定的独立 domain，
再按部署需要配置网络范围。此隔离示例不要求修改 `.bashrc` 或终止其他工作区的进程。

本机已把默认 install 中的旧 `car_description/launch` 目录可逆备份至
`log/bringup_migration/legacy_launch`；历史 Phase 8 install 保留。后续终端使用
当前 `install/setup.bash` 和 `bringup` 入口。
