# Phase 3：最小 2D A*

本文件记录 Phase 3 行为。当前代码已接入 [Phase 4 后处理](path_processing_phase4.md)：
`/plan` 输出 processed path，原始 A* path 改由 `/plan/raw_path` 调试查看；当前验证以 Phase 4 文档为准。

## 接口与启动前置条件

| 输入/输出 | 默认 topic / TF | 标准类型 |
| --- | --- | --- |
| Phase 2 inflated grid | `/plan/inflated_grid` | `nav_msgs/msg/OccupancyGrid` |
| Goal | `/goal_pose` | `geometry_msgs/msg/PoseStamped` |
| Start | `planning_frame -> base_footprint` 最新 TF | TF transform |
| Path | `/plan` | `nav_msgs/msg/Path` |
| 结果状态 | `/plan/status` | `std_msgs/msg/String`，内容为下表中的状态名 |

参数位于 `src/plan/config/astar.yaml`，可通过参数/remap 覆盖。
`planning_frame` 必须显式设置为实际地图 frame；为空时启动失败。
现有仓库尚未确认 map/localization 方案，本阶段不增加定位系统或实际机器人 TF。
部署必须已有完整的 `planning_frame -> base_footprint` TF 链。
Task 只发目标，机器人起点不从 Task、参数或 goal 消息中获取。
当前只支持 TF 位姿来源，不静默回退到 odometry 坐标系。

地图与状态使用 reliable/transient_local/depth 1；goal 与 Path 使用
reliable/volatile/depth 1，与 V1 Path 接口契约一致。
RViz 要先订阅 Path，再发送新目标；旧路径不为晚加入节点保留。

## 算法与坐标系

- A* 核心为独立 C++ 库 `astar_core`，不依赖 ROS/TF。
- 直接导入 Phase 2 inflated grid，允许的值只有 `100/0/-1`；不二次膨胀。
  `100` 禁行；`0` 通行；`-1` 沿用 Phase 2 已选择的允许 unknown 通行策略。
  Phase 2 的 `unknown_is_obstacle: true` 会将 unknown 编码为 `100`。
- 8 邻域，直线代价 `resolution`，对角代价 `sqrt(2) * resolution`，总成本单位为米。
  使用 admissible/consistent octile 启发式、最小堆、closed set 和 parent 回溯。
- 固定禁止 corner-cutting：一次对角移动两侧的正交 cell 都必须可通行。
- 起点查找最新 TF，并检查动态 TF 的时间偏差不超过 `tf_max_age_sec`。
  时间戳为零的纯静态 TF 不做动态年龄检查；合成测试允许用静态 TF 表示假机器人。
- Goal 必须有非空 frame、有限坐标和单位 quaternion。目标 frame 不同则由 TF
  按目标时间戳转换；stamp 为零使用最新变换。
- 地图必须匹配 `planning_frame`，origin 为单位 yaw-only quaternion。
  转换复用 Phase 2 的 origin 平移/yaw 逻辑。
- Path 是按顺序排列的 cell 中心；起止位置分别落在实际 TF start/goal 所在 cell 中心，
  不把目标 cell 中心伪称为精确目标坐标。
  每个 PoseStamped header 与 Path header 一致，中间朝向指向下一点，终点使用目标 yaw。
- 本阶段不做 smoothing、shortcut、resampling 或连续曲线碰撞验证。

## 状态和失败处理

| 状态 | 含义 |
| --- | --- |
| `SUCCESS` | 已生成非空、合法的 grid path；start/goal 同 cell 返回单点路径 |
| `INVALID_START` | TF 位置非有限、越界或处于 blocked cell |
| `INVALID_GOAL` | goal 格式非法、转换后越界或处于 blocked cell |
| `NO_PATH` | 合法起终点之间没有满足邻域/禁穿角规则的路径 |
| `MAP_UNAVAILABLE` | 尚无有效 inflated grid，或收到非法/不同 frame 的地图 |
| `TF_UNAVAILABLE` | 起点/目标所需 TF 缺失、无法按时间转换或动态起点 TF 过期 |

状态描述最近一次请求结果；错误日志提供具体原因。
检查顺序为地图、goal 格式、TF 转换、坐标转换和 A*；不要将 TF 缺失理解为障碍不可达。
每次失败都发布空 Path 清除旧路径，不返回部分路径。
收到新的有效地图时旧 Path 也会清空，需要重新发目标；不自动重规划。
状态本身不表示“旧路径仍有效”，消费方必须检查实际 Path 是否为空。
非法地图会清除 Planner 的地图快照，之后目标返回 `MAP_UNAVAILABLE`。
核心库只返回图搜索相关状态，`TF_UNAVAILABLE` 由 ROS wrapper 产生。

## 构建与测试

在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/phase3_astar build --base-paths src \
  --build-base build/phase3_astar --install-base install/phase3_astar \
  --packages-select plan
source install/phase3_astar/local_setup.bash
mkdir -p log/phase3_astar/ros
export ROS_LOG_DIR="$PWD/log/phase3_astar/ros"
colcon --log-base log/phase3_astar test \
  --build-base build/phase3_astar --install-base install/phase3_astar \
  --packages-select plan
colcon test-result --test-result-base build/phase3_astar --verbose
```

核心测试覆盖直线/对角/混合最优成本、同 cell、禁穿角、非法起终点、不可达、
无二次膨胀和旋转 origin；随机地图用独立的 dense Dijkstra 作最优成本对照。
ROS 测试覆盖六种状态、失败清除旧 Path、Phase 2 链路、goal frame 转换、
动态 TF 过期和每次请求重新获取 TF 起点。测试隔离在 localhost domain，
不发布任何速度命令。

## 已有地图/定位环境中的 RViz 验证

分别启动 Phase 2 grid 和 A*，planning_frame 填真实地图 frame：

```bash
ros2 launch plan planning_grid.launch.py
ros2 launch plan astar.launch.py planning_frame:="实际地图frame"
rviz2 -d src/plan/config/planning_grid.rviz
ros2 topic echo /plan/status --qos-durability transient_local
```

RViz Fixed Frame 与 planning_frame 一致，使用 2D Goal Pose 工具发送 `/goal_pose`。
观察 Inflated Grid 与绿色 AStar Path；状态应为 `SUCCESS`，路径不跨 blocked cell，
也不从障碍夹角中穿过。将目标放在障碍/地图外应得到 `INVALID_GOAL` 和空 Path。
实际定位链尚未具备时应得到 `TF_UNAVAILABLE`，不能用假 TF 启动实际自主导航。

## 独立合成验证（不代表实际 localization）

所有终端先 source 上面的环境，并统一设置一个不与已有仿真共用的 ROS domain。
下面 `grid_debug` 与静态 start 只用于此合成地图：

```bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
```

分别启动：

```bash
ros2 launch plan planning_grid.launch.py
ros2 launch plan astar.launch.py planning_frame:=grid_debug use_sim_time:=false
ros2 run tf2_ros static_transform_publisher \
  --x 1.25 --y 1.25 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id grid_debug --child-frame-id base_footprint
rviz2 -d src/plan/config/planning_grid.rviz
```

发布测试地图；先在 RViz 或另一个终端订阅 Path，再发目标：

```bash
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable \
  /map nav_msgs/msg/OccupancyGrid \
  '{header: {frame_id: grid_debug}, info: {resolution: 0.5, width: 9, height: 9, origin: {orientation: {w: 1.0}}}, data: [0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,100,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0]}'

# 另一个终端运行订阅，再发送目标。
ros2 topic echo /plan --once
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: grid_debug}, pose: {position: {x: 3.25, y: 3.25}, orientation: {w: 1.0}}}'
ros2 topic echo /plan/status --once --qos-durability transient_local
```

预期绕开中心的膨胀障碍，Path frame 为 `grid_debug`；RViz 配置已包含 Path 和 Goal 工具。
本阶段没有 LQR、底盘命令、Hybrid A*、Nav2 planner plugin、机械臂或深度相机逻辑。
