# map1 地图上的实车定点导航

使用 `car_ws` 原有 `src/urdf/urdf/car_urdf.urdf` 模型，不修改 URDF 文件。
使用 `/home/dx/tzcup/car_ws/map1/room.yaml` 和同目录 `room.pgm`，
在 RViz 中逐次指定目标点，经 A*、路径处理、LQR 和 Safety Gate 完成导航。
这里的“定点巡航”采用逐次指定目标的方式，不设置自动循环点位列表。

## 启动范围与外部前提

`navigation_real.launch.py` 默认启动原模型的 robot_state_publisher、
显示用 joint_state_publisher、地图服务、AMCL、定位 lifecycle manager、
planning_grid、A*、LQR、Safety Gate 和可选 RViz。所有这些节点强制使用
`use_sim_time: false`，不依赖 `/clock`。

Agent、雷达驱动和必要的 odom2tf 由现有工作区独立启动。
本入口不会启动 Gazebo、仿真差速控制器或旧 Nav2 导航栈。
原 URDF 保留的 Gazebo 标签仅作为模型内容读取，不会因此运行 Gazebo 插件。

已确认接口：

| 项目 | 配置 |
| --- | --- |
| ESP32 | `/dev/ttyUSB0`，micro-ROS 串口，115200 |
| A1M8 | `/dev/ttyUSB1`，约 7.76 Hz，启动时指定 frame `laser_link` |
| 模型 | `car_description` 包中的 `urdf/car_urdf.urdf` |
| 地图 | `map1/room.yaml`、`map1/room.pgm`；221 × 338，0.05 m/格 |
| 底盘命令 | `/cmd_vel`，`geometry_msgs/msg/Twist`，ESP32 BEST_EFFORT 订阅 |
| 里程计 | `/odom`，`nav_msgs/msg/Odometry`，BEST_EFFORT 发布 |
| 里程计 frame | `odom -> base_footprint` |
| 导航目标 | RViz `/goal_pose`，`geometry_msgs/msg/PoseStamped` |
| 路径 | `/plan`，`nav_msgs/msg/Path`，以及配套 `/plan/profile` |
| 默认控制链 | LQR `/cmd_vel_raw` -> Safety Gate `/cmd_vel` -> ESP32 |
| 显式旁路控制链 | `use_safety_gate:=false enable:=true`：LQR `/cmd_vel` -> ESP32 |

完整 TF 应为：

```text
map --AMCL--> odom --里程计 TF--> base_footprint
                                  |
                             原有 URDF
                                  |
                             base_link -> laser_link
```

AMCL 在地图激活、扫描与初始定位就绪后提供 `map -> odom`。不要用静态
`map -> odom` 代替定位。Planner 与 LQR 自行从 TF 获取当前位置。

若底层只发布 `/odom` 而不发布里程计 TF，可运行已有的
`ros2 run src odom2tf`；若这条 TF 已有发布方，则不要重复启动。
原模型的 `laser_joint` 相对 base_link 位移为 `[0.155, 0, 0.1855] m`，
yaw 为 `3.14 rad`，从 base_footprint 看高度为 `0.2825 m`。
沿用模型时仍须核实真实安装符合这些外参。更改 frame_id 仅统一名称，
不会纠正真实位置或朝向差异；扫描与地图不对齐时先检查定位与外参。

## Gazebo 与实车冲突

实车入口的 `start_sim` 固定只接受 `false`，传入 `true` 会拒绝启动。
原 `navigation.launch.py` 和 `sim.launch.py` 保持不变，仍用于仿真。

在同一 ROS domain 中，Gazebo 与 ESP32 会同时订阅 `/cmd_vel`，仿真还会与
实车共享 `/odom`、`/scan` 和 TF，因此存在命令串扰及定位数据冲突。
本入口不会关闭已经在其他终端运行的 Gazebo；实车启动前应退出它，
或将完整仿真链放入不同的 `ROS_DOMAIN_ID`。仅改变 `/cmd_vel` 不足以隔离
其他传感器和 TF。所有实车节点以及 ESP32 participant 需要处于一致的 domain。

自动导航运行时，默认模式的 Safety Gate、旁路模式的 LQR 应分别是最终速度
话题的唯一发布方；退出键盘遥控和旧 Nav2 导航控制器。
默认模式即使 `enable:=false`，Safety Gate 也会持续发布零速度，
所以不要同时让键盘向同一话题发送运动指令。

## 地图填写和参数文件

默认加载仓库 `map1/room.yaml` 的安装副本，不回退到仿真地图。
也可以显式使用源码地图，便于后续直接更新：

```bash
ros2 launch car_description navigation_real.launch.py \
  map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml
```

当前 YAML 的 `image: room.pgm` 相对该 YAML 所在目录解析；分辨率为 0.05 m，
原点为 `[-4.34, -3.17, 0]`，保留地图原有内容。
路径留空或文件不存在时，在启动任何导航节点前报错；图片和地图内容由地图
服务进一步校验。CMake 已增加 map1 安装规则，默认入口使用安装副本。

| 文件 | 用途 |
| --- | --- |
| `navigation_real.yaml` | 实车话题、frame、误差预算和 Safety Gate 覆盖参数 |
| `amcl_real.yaml` | AMCL 定位；无固定初始位姿，通过 `/initialpose` 初始化 |
| `motion_limits_real.yaml` | A*、LQR 与 Safety Gate 共享的运动限制 |
| `navigation_real.rviz` | 地图、膨胀栅格、扫描、路径和定位/目标工具 |

保留 plan/controller 原有算法参数文件，实车覆盖参数随后加载，运动限制再加载，
最后强制真实时间及 launch 的运动使能和输出话题。
`enable` 通过启动参数设置；修改 YAML 中的同名值不会覆盖启动参数。

当前限速文件为巡航 0.20 m/s、最大 0.30 m/s、最大角速度 0.60 rad/s；它们是
低速调试起点，不是实测能力。车体半径沿用 0.355 m 的现有包络，安全余量
0.25 m，最大跟踪误差 0.20 m；实车尺寸与误差预算需要复核。
Safety Gate 停车/释放阈值沿用 0.20/0.65 m，从雷达原点计算，不是车头净空。
实车启用有限缺失回波模式：正无穷仍是未知距离，不参与最近障碍距离计算。
初始条件为前方有限有效比例至少 0.80、相邻有限射线及扇区两端最大间隔
不超过 0.105 rad（约 6°）；这些是离线/隔离测试起点，尚未通过实车验收。
全无回波、缺失过多、连续盲区过大、NaN、负无穷和越界距离都会停车。
仿真保留原有正无穷表示最大量程无障碍的配置，实车不启用该语义。
`scan_timeout_sec: 0.30` 保留现有超时，需检查真实抖动和消息时间戳。

## 构建和验证

本次包含 controller 的 C++ 修改，需要同时构建控制节点和实车资源，
仅复制 car_description 配置无法更新已安装的 Safety Gate 可执行程序：

```bash
cd /home/dx/tzcup/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to controller car_description
source install/setup.bash
colcon test --packages-select controller plan
colcon test-result --verbose
ros2 launch car_description navigation_real.launch.py --show-args
git diff --check
```

CMake 安装 config/launch/urdf，并新增 map1 地图安装条目。
`--show-args` 只检查启动参数，不证明硬件或闭环导航通过。
后续各导航终端使用本次 `install/setup.bash`。若以前加载过
`install/real_navigation_config/local_setup.bash`，请打开新终端按下述命令
加载，避免旧的配置 overlay 优先于本次安装版本。
`colcon test-result --verbose` 也会汇总 build 内的历史独立构建结果；
只查看本轮两个包时，可分别运行：

```bash
colcon test-result --test-result-base build/controller/test_results --verbose
colcon test-result --test-result-base build/plan/test_results --verbose
```

## 分终端启动步骤

各终端保持相同、且与已连通的 ESP32 participant 一致的 ROS domain 设置。
下面每个持续运行的命令分别使用一个终端；已有对应节点时不重复启动。

### 终端 1：ESP32 Agent

```bash
source /opt/ros/humble/setup.bash
source /home/dx/zip/agent_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 -b 115200 -v4
```

必要时按一下 ESP32 的 EN/RST，观察 `session established`、`datareader created`、
`datawriter created`。`-v6` 会打印十六进制数据，调试连接时用 `-v4` 更易观察。
不要用仓库中默认 UDP 的 Agent launch 替代这条串口命令。

### 终端 2：A1M8 雷达

```bash
source /opt/ros/humble/setup.bash
source /home/dx/zip/rplidar_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/ttyUSB1 serial_baudrate:=115200 frame_id:=laser_link
```

使用不带 `view_` 的入口，避免额外打开雷达专用 RViz。
此工作区当前保留安装产物但源码目录为空，先使用现有安装，不直接重新构建。
确认 `/scan` 持续输出、frame 为 `laser_link`。如果旧驱动仍使用 `laser`，
退出旧实例，按上述命令重启；USB 重新插拔后仍需核对设备编号。

### 模型随导航入口自动启动

原 `car_urdf.urdf` 已由实车导航入口发布，不再单独运行 agent_ws 的
`ros2 launch src robot.launch.py`（另一套模型），也不使用当前固定为仿真时间的
`car_description display.launch.py`。原模型文件与原仿真入口不变。
实车 RViz 已配置 `/initialpose` 和 `/goal_pose`，不额外打开旧模型展示 RViz。

已有进程发布同一原模型时，传 `publish_robot_model:=false`，避免重复发布。
默认 `publish_joint_states:=true` 只提供显示用关节位置，不是轮编码器反馈或里程计；
若已有真实 `/joint_states` 发布者，传 `publish_joint_states:=false`。

### 终端 3：里程计 TF

仅当当前没有节点发布 `odom -> base_footprint` 时执行：

```bash
source /opt/ros/humble/setup.bash
source /home/dx/zip/agent_ws/install/setup.bash
ros2 run src odom2tf
```

可执行程序名是 `odom2tf`，不是 `tf2`。它只把已有 `/odom` 转成 TF，
不会生成里程计数据。

### 终端 4：原 URDF + map1 地图 + 导航，先禁止运动

```bash
source /opt/ros/humble/setup.bash
source /home/dx/tzcup/car_ws/install/setup.bash
ros2 launch car_description navigation_real.launch.py \
  map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml \
  enable:=false cmd_vel_topic:=/real_navigation_check/cmd_vel
```

此例直接读取 map1 地图，同时发布原模型；最终速度发往调试话题，不连接底盘。
默认 `rviz:=true`；只运行节点时可加 `rviz:=false`。`start_sim` 默认为 `false`，
不需要额外填写，也不能在本入口中改成 `true`。

### 终端 5：检查与发布目标

```bash
source /opt/ros/humble/setup.bash
```

检查地图/AMCL 激活、真实时间、共享限速和传感器数据：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 param get /map_server yaml_filename
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /planner use_sim_time
ros2 param get /lqr_controller use_sim_time
ros2 param get /safety_gate use_sim_time
ros2 param get /planner v_max
ros2 param get /lqr_controller v_max
ros2 param get /safety_gate v_max
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

`tf2_echo` 会持续运行，检查后按 Ctrl+C，再运行下一条命令。

RViz 使用 `2D Pose Estimate` 初始化后确认 `map -> base_footprint` 与扫描地图
对齐，再通过目标工具向 `/goal_pose` 发布目标，检查 `/plan` 与 `/plan/profile`。
禁止运动时，最终输出应为零；`/cmd_vel_raw` 仍可能有 LQR 的非零候选命令。

可继续检查：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /plan --once
ros2 topic echo /plan/profile --once
ros2 topic echo /real_navigation_check/cmd_vel --once
ros2 topic echo /controller/lqr/status --once
ros2 topic echo /controller/safety/status --once
```

Path/Profile 是 volatile 数据：先启动相应 echo，再在 RViz 发布一次新目标，
以免在路径已发布完后等待下一次消息。状态话题则保留最新状态。

### 先验证速度转发，输出到调试话题

退出终端 4 的导航实例，使用相同环境启动下面的实例；不同时运行两个 Gate：

```bash
ros2 launch car_description navigation_real.launch.py \
  map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml \
  enable:=true cmd_vel_topic:=/real_navigation_check/cmd_vel
```

重新设置初始位姿和目标。检查车头实际障碍在 RViz 中是否位于车体前方，
并连续观察多帧扫描，确认雷达朝向、扫描与地图对齐。若方向不符，先处理
实际安装外参，不能只改 frame 名称或放宽扫描阈值。

在其他终端依次检查参数和输出；echo/hz 持续运行时按 Ctrl+C 后再执行下一条：

```bash
ros2 param get /safety_gate enable
ros2 param get /safety_gate allow_positive_infinity
ros2 param get /safety_gate allow_sparse_positive_infinity
ros2 param get /safety_gate min_valid_fraction
ros2 param get /safety_gate max_valid_beam_gap
ros2 topic hz /scan
ros2 topic echo /cmd_vel_raw
ros2 topic echo /real_navigation_check/cmd_vel
ros2 topic echo /controller/safety/status
```

定位与扫描正常、LQR 持续给出非零候选速度时，Gate 应显示 `CLEAR`，
调试输出应与候选速度一致。该实例只发布调试速度，小车静止时不能完成
真正的路径跟踪，到跟踪误差或其他保护条件触发时仍可能停车。
`enable:=false` 会持续输出零，无法用来证明速度转发已经通过。

节点限频日志报告 `reason`、`finite`、`valid_fraction` 和 `max_valid_gap`。
`INVALID_BEAM` 表示严格模式中的非有限值或非法距离；
`SECTOR_COVERAGE` 表示扫描原始角度覆盖不足；`NO_FINITE_RETURN` 表示无有限回波；
`VALID_FRACTION` 和 `VALID_BEAM_GAP` 分别表示有效比例不足或连续缺失过大。
`max_valid_gap` 单位为 rad，包含前方扇区两端。元数据损坏时统计量可能尚未计算。
若连续扫描仍失败，保留日志并检查障碍、安装和传感器，不自动放宽阈值。

### 开启实车低速运动

完成车体/雷达外参、扫描语义、定位、制动和通信断连停车核实后，退出上面的实例，
再显式开启低速运动，使用默认真实 `/cmd_vel`：

```bash
ros2 launch car_description navigation_real.launch.py \
  map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml \
  enable:=true cmd_vel_topic:=/cmd_vel
```

`enable` 是启动时只读参数，不能依赖 `ros2 param set` 在线切换。
重启导航后重新设置初始位姿，确认定位，再指定第一个目标。
到点停车且 LQR 状态为 `GOAL_REACHED` 后，在 RViz 指定下一个目标，重复即可逐点导航。
未到点时发布新目标会替换当前目标；当前没有自动多点队列。
本次配置修改不代表已通过实车行驶、避障或制动验收。
现场验收应确认 `/cmd_vel` 唯一发布方为 Gate、ESP32 订阅存在，目标后
`/cmd_vel_raw` 与 `/cmd_vel` 出现对应非零值，底盘运动时 `/odom` 随之变化，
到点停止；再验证近障碍和扫描断流时最终速度归零。

### 关闭 Safety Gate，LQR 直接输出

本模式按用户要求跳过 Gate 的扫描有效性、实时障碍停车、扫描/原始速度
超时与 Gate 限值检查。LQR 自身的路径、TF、里程计、跟踪误差、动态预算及
watchdog 检查继续生效；AMCL 仍需要雷达。当前定位跳动引起的
`TRACKING_ERROR` 不会因为旁路 Gate 自动消失。

先构建更新后的 LQR 节点和入口，然后退出旧导航终端中的整个 launch。
不能保留旧 Gate，否则它的零速度会与 LQR 争用同一底盘话题。

```bash
cd /home/dx/tzcup/car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select controller car_description
source install/setup.bash
```

旁路验证时先指定调试输出：

```bash
ros2 launch car_description navigation_real.launch.py \
  map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml \
  use_safety_gate:=false enable:=true \
  cmd_vel_topic:=/real_navigation_check/cmd_vel
```

需要连接 ESP32 时，将命令改为：

```bash
ros2 launch car_description navigation_real.launch.py \
  map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml \
  use_safety_gate:=false enable:=true cmd_vel_topic:=/cmd_vel
```

重新进行初始定位，再在 RViz 发布目标。此时不启动 `safety_gate`，LQR 的
`raw_cmd_topic` 被入口最后覆盖为 `cmd_vel_topic`；不再发布 `/cmd_vel_raw`。
节点新增只读 `allow_direct_cmd_vel` 参数，默认 false，实车旁路入口将其设为
true，保留原有话题碰撞检查。单改 YAML 的 raw 话题不会隐式开启该许可。
旁路与 `enable:=false` 同时指定时，入口在启动节点前拒绝执行。

检查实际接线；旁路时 `/safety_gate` 应不存在，`/cmd_vel` 唯一发布方应为
`lqr_controller`，订阅方包含 `motion_control_node`：

```bash
ros2 node list
ros2 param get /lqr_controller allow_direct_cmd_vel
ros2 param get /lqr_controller raw_cmd_topic
ros2 topic info /cmd_vel --verbose
ros2 topic echo /controller/lqr/status
```

恢复 Gate 时，先退出旁路 launch，再以 `use_safety_gate:=true enable:=true`
启动。默认 `use_safety_gate=true`，原仿真入口和默认实车控制链保持原行为。

本次旁路构建通过。隔离入口测试 3 项通过，LQR 定向回归 18 项通过，包含
显式直连、默认拒绝底盘直连、话题冲突拒绝、里程计超时停车和原有跟踪用例。
没有替换正在实车 domain 中运行的旧实例，也未进行旁路实车行驶验收。
可在上述构建和 source 后复验入口：

```bash
python3 -m pytest -q src/urdf/test/test_navigation_real_routing.py
```

## 如何修改参数

优先修改 `src/urdf/config/` 下的实车文件，不修改安装目录中的副本，
也不为实车调参直接改变仿真配置。

| 需求 | 文件/参数 | 注意事项 |
| --- | --- | --- |
| 更换地图 | launch `map_yaml` | 默认安装的 map1/room.yaml，也可传源码地图绝对路径 |
| 发布原模型 | launch `publish_robot_model` | 默认 true，已有同一模型发布方时设 false |
| 显示用关节状态 | launch `publish_joint_states` | 默认 true，已有真实关节状态时设 false |
| 允许运动 | launch `enable:=true` | 默认 false，修改后重启该入口 |
| 关闭/恢复 Gate | launch `use_safety_gate` | 默认 true；false 时 LQR 直接输出，必须同时 enable:=true |
| 调试速度输出 | launch `cmd_vel_topic` | 默认 `/cmd_vel`，可改成不被底盘订阅的话题 |
| 降低巡航与最高速度 | `motion_limits_real.yaml` 的 `v_nominal`、`v_max` | 当前 0.20/0.30 m/s，满足 `0 < v_nominal <= v_max` |
| 调整转弯速度 | 同文件 `w_max` | 单位 rad/s，由 Planner、LQR、Safety Gate 一起加载 |
| 调整加减速 | 同文件 `linear_acceleration`、`linear_deceleration` | 单位 m/s²，参数必须大于零 |
| 调整角加速度 | 同文件 `angular_acceleration` | 单位 rad/s² |
| 车体尺寸与余量 | `navigation_real.yaml` 中 planning_grid 的 `robot_radius`、`safety_margin` | 单位 m，半径按实际完整包络核实 |
| 跟踪误差上限 | 同文件 lqr_controller 的 `max_tracking_error` | 单位 m，不通过放宽误差掩盖定位故障，需小于安全余量 |
| 雷达 frame | 同文件 safety_gate 的 `scan_frame` | 必须与扫描 header 和实际 TF 一致 |
| 扫描超时 | 同文件 safety_gate 的 `scan_timeout_sec` | 单位 s，根据频率、抖动和停机距离核实 |
| 停车/恢复距离 | 同文件 safety_gate 的 `stop_distance`、`release_distance` | 单位 m，恢复距离必须大于停车距离，从雷达原点计算 |
| 有限缺失模式 | 同文件 `allow_sparse_positive_infinity` | 实车 true；默认 false，旧配置保持原行为 |
| 最低有效回波比例 | 同文件 `min_valid_fraction` | 初始 0.80，分母是前方扇区的全部射线，有限且量程内才计入分子 |
| 最大有限射线间隔 | 同文件 `max_valid_beam_gap` | 初始 0.105 rad，检查相邻有限射线及两侧边缘，不能用比例代替连续盲区检查 |
| 仿真无回波语义 | 同文件 `allow_positive_infinity` | 实车 false；true 将正无穷按最大量程处理，与有限缺失模式互斥 |
| 定位 frame、粒子数、更新阈值 | `amcl_real.yaml` | 保持 map/odom/base 语义一致，初始位姿由 RViz 提供 |
| RViz 显示与目标工具 | `navigation_real.rviz` | 目标工具使用 `/goal_pose`，不能改成依赖 Nav2 action 的目标工具 |

如果需要覆盖基础文件里的其他参数，可把它添加到 `navigation_real.yaml`
对应节点的 `ros__parameters` 下。例如 LQR 的 `lookahead_distance`、
`odom_max_age_sec`，或 Safety Gate 的 `max_beam_gap`。
不要在这里重复设置 `v_max` 等共享运动限制；它们随后会被运动限制文件覆盖。

例如，把巡航/最高速度进一步降低：

```yaml
/**:
  ros__parameters:
    v_nominal: 0.10
    v_max: 0.15
```

这只是要修改的两个字段，其他运动限制字段必须保留。

修改源码后，重新执行本文构建命令，加载 `install/setup.bash`，
退出旧导航实例后重新启动。修改 C++ 后必须重新构建 controller。
运动限制、LQR 和 Safety Gate 的许多参数为只读，不能假定在线设置会生效。
若只修改外部地图 YAML/图片，不需要重新构建包，但要重启入口加载地图并重新定位。

也可以直接加载源码配置，免去仅为 YAML 改动重新复制安装文件的步骤：

```bash
ros2 launch car_description navigation_real.launch.py \
  map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml enable:=false \
  navigation_params_file:=/home/dx/tzcup/car_ws/src/urdf/config/navigation_real.yaml \
  amcl_params_file:=/home/dx/tzcup/car_ws/src/urdf/config/amcl_real.yaml \
  motion_limits_file:=/home/dx/tzcup/car_ws/src/urdf/config/motion_limits_real.yaml
```

仍然需要重启旧节点；修改 launch 本身或默认 RViz 文件时应重新构建。

## 常见问题的判断顺序

- `/cmd_vel` 没有订阅者、`/odom` 没有发布者：先看 Agent 会话和实体创建日志。
  键盘可创建速度发布端，odom2tf 可创建里程计订阅端，它们不代表底层已接通。
- 两个端点已有但 echo 无数据：检查实际 QoS、频率、时间戳；里程计检查可显式
  使用 `--qos-reliability best_effort`。
- `map_server` 未 active：检查地图路径、图片文件和 YAML 内容。
- AMCL 无全局 TF：检查 `/initialpose`、`/scan`、里程计 TF 和雷达安装 TF。
- 雷达仍为 `laser`：退出旧驱动，按本文命令指定 `frame_id:=laser_link` 重启。
- 重复 TF 或机器人跳动：检查是否同时运行旧模型发布器、Gazebo 或多个 odom2tf。
- Safety Gate 持续停车：查看 `/controller/safety/status`，核实 enable、扫描 frame、
  无效回波、超时、障碍物和速度上限；不直接绕过 Gate。
- 发布目标后无路径：查看 `/plan/status`，检查定位、地图与膨胀后是否仍有可行通路。
- 有路径但 LQR 停车：查看 `/controller/lqr/status`，检查 Path/profile 配对、
  里程计/TF 时效和实际跟踪误差。

## 之前模型与地图接入的验证记录

- car_description 构建和 `--show-args` 通过；安装的地图与 map1 源文件一致，
  安装的原 URDF 与源码一致。
- launch 展开检查确认包含可选模型发布节点，10 个节点均使用真实时间，
  默认禁止运动、拒绝 Gazebo 请求。
- 在隔离 domain 219 中，`rviz:=false`、禁止运动且输出为调试话题时，
  9 个后台节点的真实时间参数检查通过；地图服务与 AMCL 均为 active。
- 已收到 map1 对应的 221 × 338 栅格、0.05 m 分辨率及原始地图原点，
  并查到原模型的 `base_footprint -> laser_link` 静态 TF。
- 已确认 Planner/LQR 巡航 0.20 m/s，Planner/LQR/Gate 最大速度均为
  当前限速文件的 0.30 m/s；调试输出为零，没有实际 `/cmd_vel` 发布者，
  没有启动 Gazebo/controller_manager。

运行日志：`log/real_navigation_config/runtime_model_map1_validation.log`。
执行环境有 UDP socket 权限告警，本机参数服务与调试话题检查成功；
USB 实车、真实定位、RViz GUI、跨机 DDS 和实际行驶仍需现场验收。

## 本次缺失回波适配的验证记录（2026-11-04）

- `colcon build --packages-up-to controller car_description` 成功，4 个包完成；
  安装的实车 YAML 与源码一致，`--show-args` 确认默认 `enable=false`。
- Safety Gate 核心 19 个测试、节点 31 个测试全部通过。节点测试使用隔离
  ROS domain 和 `/gate_test/out`，没有向真实 `/cmd_vel` 发运动命令。
- 实测扫描夹具保留全部 720 个距离值；旧严格策略拒绝，新策略在约定阈值下
  接受。根目录原始 `scan_once.yaml` 保留，单帧通过不代表实车阈值验收。
- plan 结果汇总为 195 项、0 失败；controller 为 167 项、5 失败。
  这些汇总含 lint 和跳过项目，具体结果见各包 test_results。
- controller 剩余失败是未修改的 DWA 文件格式检查（flake8 1 项、
  uncrustify 3 项），以及 1 个未修改的 LQR 节点用例。
  该用例单独复现后，日志为 `Dynamic budget requires continuous nodal curvature`：
  圆弧测试 profile 未提供当前动态预算要求的连续曲率标记。
  本次保留其代码和结果，不通过修改 LQR 或跳过用例掩盖失败。
- XML 检查最初因网络限制无法获取 schema；随后使用
  [ROS 官方仓库 schema](https://github.com/ros-infrastructure/rep/blob/master/xsd/package_format3.xsd)
  的临时本地副本，通过 `XML_CATALOG_FILES` 重跑两个包的 xmllint，均通过。
  仓库 package.xml 与测试开关没有修改。

诊断记录位于 `log/real_scan_validation/`，包括分包测试汇总和
`lqr_diagnostic.txt`；该目录属于忽略的运行产物。未完成 USB 实车行驶、
连续实测扫描、朝向核实和制动验收，不能据此宣称实车闭环导航已经通过。

修改保留在 Git 更改区，不执行 git add 或 git commit。
