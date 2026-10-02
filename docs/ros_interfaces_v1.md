# ROS 接口契约 V1

本文件冻结轻量导航 baseline 的模块边界、标准消息、默认 topic 和坐标系语义。
根目录 `AGENTS.md` 提供长期规则和 Phase 0 事实。
本阶段交付接口契约、可安装的参数文件及标准消息/TF 依赖；当前业务包仍为骨架，
没有新增 Task、Planner、LQR 或 Safety Gate 可执行节点。

## 事实与新约定

- 现有底盘配置：`/cmd_vel`，非 stamped 速度命令；里程计 remap 到 `/odom`。
- 现有 LiDAR 配置：`/scan`、`sensor_msgs/msg/LaserScan`、`laser_link`。
- 已确认 frame：`odom`、`base_footprint`、`base_link`、`laser_link`。
- 以下名称是本阶段冻结的新接口：`/goal_pose`、`/map`、`/plan`、`/cmd_vel_raw`。
  它们不表示当前已经存在相应 publisher。
- map frame、定位实现、选用地图尚未确认。本契约不设置默认 `map` frame，
  不新增 map server、定位节点或 `map -> odom` 静态变换。
- 以上现有接口来自仓库配置。运行仿真时仍应验证实际 remap、生效 topic 和 TF，
  尤其是历史遥控说明中使用的 `/diff_drive_controller/cmd_vel_unstamped`。

## 最终接口表

类型采用 ROS 2 CLI 写法；例如 `nav_msgs/msg/Path` 就是 `nav_msgs/Path`。

| 接口 | 默认 topic / TF | 类型 | 生产方 -> 消费方 | 参数 |
| --- | --- | --- | --- | --- |
| Goal | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Task / 外部 goal -> Planner | `goal_topic` |
| Map | `/map` | `nav_msgs/msg/OccupancyGrid` | 地图来源（未配置）-> Planner | `map_topic` |
| Path | `/plan` | `nav_msgs/msg/Path` | Planner / Path Processing -> LQR | `path_topic` |
| 当前 pose（首选）| TF：`planning_frame -> base_footprint` | TF transform | localization / odometry TF -> Planner、LQR | `planning_frame`、`base_frame`、`pose_source` |
| 当前状态（可选）| `/odom` | `nav_msgs/msg/Odometry` | 现有差速控制器 -> Planner、LQR | `odom_topic`、`odom_frame`、`allow_odom_fallback` |
| 2D LiDAR | `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo ray sensor -> Safety Gate | `scan_topic`、`scan_frame` |
| Raw command | `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | LQR -> Safety Gate | `raw_cmd_topic` |
| 底盘速度命令 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Safety Gate -> 现有差速控制器 | `cmd_vel_topic` |

节点名约定为 `task`、`planner`、`lqr_controller`、`safety_gate`；
Planner 对应 `plan` 包，LQR 和 Safety Gate 对应 `controller` 包。
这些是接口角色的命名约定，当前没有对应 executable。
`common_msg` 仅承载共享配置，本阶段不生成自定义消息。
Task 只提供目标，不发送 `start_pose` 或机器人实时起点。

## Frame 和消息语义

- `planning_frame` 必须由部署配置显式填写，并在 Planner 和 LQR 中保持一致。
  参数为空、地图未就绪或所需 TF 不可用时，自动导航不能启动。
- Map 的 `header.frame_id`、Path 的 `header.frame_id` 及每个路径点的
  `header.frame_id` 必须与 `planning_frame` 一致。
- Goal 必须携带有效的 frame、时间戳和四元数；若其 frame 与 `planning_frame`
  不同，由 Planner 通过 TF 转换目标。TF 不可用时拒绝目标，不猜测坐标系。
- Planner 自行读取当前 pose；LQR 也读取当前 pose。优先使用 TF 查找机器人
  在 `planning_frame` 中的位姿，`pose_source` 固定为 `tf`。
- `/odom` 可提供速度或在显式启用 `allow_odom_fallback` 后提供位姿。
  必须检查 `header.frame_id` 和 `child_frame_id`；只有在规划 frame 是实际 odom
  frame，或有有效 TF 将其转换到规划 frame 时，才能将 odometry pose 用于导航。
  `odom` 位姿不得直接当作 map 位姿。
- 两级 `Twist` 都表示机器人底盘坐标系下的速度：使用 `linear.x` 和 `angular.z`，
  其余分量为零。LQR 输出与底盘输出必须使用不同 topic。
- 自动模式下 Safety Gate 是最终底盘命令的唯一发布方；遥控与自动模式不能同时
  直接向底盘发布。过期/无效 raw command 或 LaserScan、缺失必要 TF 时必须停止，
  这些行为属于 Safety Gate 的接口要求，本阶段不实现安全逻辑。

## QoS 契约

| 数据 | QoS |
| --- | --- |
| Goal、Path | reliable、volatile、keep_last(1) |
| Map（生产方和消费方）| reliable、transient_local、keep_last(1)，允许晚加入节点获取地图 |
| LaserScan、Odometry 订阅 | best_effort、volatile、keep_last(5)，兼容传感器数据发布方 |
| Raw command、底盘命令 | reliable、volatile、keep_last(1)，不保留历史速度命令 |
| TF | 采用 tf2 的标准 `/tf`、`/tf_static` QoS |

## 参数和 remap

参数文件：`src/common_msg/config/navigation_interfaces.yaml`。
安装位置：`share/common_msg/config/navigation_interfaces.yaml`。
节点 wrapper 需要声明并读取自己 section 中的参数；topic 名称来自参数，
ROS remap 可在部署时继续覆盖。YAML 本身不会创建 topic，也不会创建节点。

`use_sim_time: true` 用于当前 Gazebo 环境。`planning_frame: ""` 是强制配置标记，
不是可运行的规划 frame 默认值；部署时应覆盖为实际地图 frame，
或经明确选择的 odom-only frame，并满足上述 TF 和地图 frame 契约。
有 namespace 的节点需要对应调整 YAML 的节点选择器。

加载方式供后续 wrapper 使用：

```python
Node(
    package=module_package,
    executable=node_executable,
    name=contract_node_name,
    parameters=[interfaces_yaml, {"planning_frame": confirmed_planning_frame}],
)
```

命令行 remap 示例（在对应 wrapper 实现后使用）：
`--ros-args --params-file <interfaces_yaml> -r /goal_pose:=/mission/goal`。
生产者/消费者必须使用一致的参数或 remap。

## 数据流

```text
Task / 外部 Goal -- PoseStamped --> Planner <-- OccupancyGrid -- 地图来源
                                     ^
                                     | TF pose（必要时 Odometry）
                                     v
                           A* -> Path Processing
                                     |
                               /plan (Path)
                                     v
                                LQR <-- TF pose / Odometry
                                     |
                            /cmd_vel_raw (Twist)
                                     v
                              Safety Gate <-- /scan (LaserScan)
                                     |
                              /cmd_vel (Twist)
                                     v
                        现有 diff_drive_controller
```

此图定义模块连接关系，算法和节点不在本阶段实现。
预留深度相机 frame 仍为 `camera_1_link`；本阶段无 camera sensor/plugin、
RGB/depth/PointCloud2 节点或相关依赖。

## 构建和验证

在仓库根目录使用独立的阶段构建目录，避免混用既有 build/install 中的历史包：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/phase1_interfaces build --base-paths src \
  --build-base build/phase1_interfaces --install-base install/phase1_interfaces \
  --packages-select common_msg task plan controller
source install/phase1_interfaces/local_setup.bash

mkdir -p log/phase1_interfaces/ros
export ROS_LOG_DIR="$PWD/log/phase1_interfaces/ros"
git diff --check -- src/common_msg src/task src/plan src/controller
git diff --no-index --check /dev/null docs/ros_interfaces_v1.md
git diff --no-index --check /dev/null src/common_msg/config/navigation_interfaces.yaml
ros2 pkg prefix --share common_msg
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show nav_msgs/msg/OccupancyGrid
ros2 interface show nav_msgs/msg/Path
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show sensor_msgs/msg/LaserScan
ros2 interface show geometry_msgs/msg/Twist

python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import rclpy
config = Path(get_package_share_directory('common_msg')) / 'config/navigation_interfaces.yaml'
assert config.read_bytes() == Path('src/common_msg/config/navigation_interfaces.yaml').read_bytes()
rclpy.init(args=['--ros-args', '--params-file', str(config)])
rclpy.shutdown()
print('Installed ROS parameter file: PASS')
PY

colcon --log-base log/phase1_interfaces test \
  --build-base build/phase1_interfaces --install-base install/phase1_interfaces \
  --packages-select common_msg task plan controller
colcon test-result --test-result-base build/phase1_interfaces --verbose
```

已有仿真运行后，在同一 ROS domain 中执行以下只读检查：

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic info /odom --verbose
ros2 topic info /scan --verbose
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

`tf2_echo` 持续运行，用 Ctrl+C 结束。新约定 topic 的端到端行为需要对应节点
实现后验证；本阶段不以 topic 已存在或机器人自主运动作为验收结果。
