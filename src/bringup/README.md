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
原 URDF 的几何和位置上下限保持；
这些参数尚未核对实机规格。Gazebo 的 JTC 使用位置状态初始化，确保首次持位目标来自
配置初值；控制器依次加载机械臂、夹爪、平移关节、状态广播和底盘。

联合入口还用 `config/gazebo_arm_physics.yaml` 生成临时世界文件，并在退出时清理。
对当前模型，ODE `world` 求解器配合轻微约束柔顺度解决了 `quick` 求解器下的高速越界。
世界场景、原世界文件和底盘单独仿真入口保持原配置；`physics_params_file:=...`
可以覆盖联合仿真的物理设置，`world:=...` 需提供单个 ODE physics 的 SDF 世界。
轨迹执行验收不等于物体接触抓取或实机力控验收。

联合入口默认只组合仿真与 MoveIt。需要导航时，在另一个同 domain 的终端复用它：

```bash
ros2 launch bringup gazebo_moveit.launch.py
# 另一个终端，source 同一环境后执行：
ros2 launch bringup navigation.launch.py start_sim:=false rviz:=false
```

该组合仍需 RViz 设置有效 `/initialpose` 并确认地图与世界对齐，导航命令继续经过
Safety Gate。MoveIt RViz 默认使用 `base_footprint`；地图初始化可另开 RViz 并加载
`car_description/config/navigation.rviz`，其 Fixed Frame 为 `map`：

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share car_description)/config/navigation.rviz" \
  --ros-args -p use_sim_time:=true -r __node:=navigation_rviz
```

导航完成后的抓取任务调度由后续任务节点实现。

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
