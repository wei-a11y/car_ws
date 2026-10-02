# Phase 4：轻量 Path Post-processing

## 数据流与接口

`Phase 2 inflated grid + TF start + PoseStamped goal -> A* -> 去重/去共线 -> 可选 LOS shortcut -> 弧长重采样 -> yaw -> Path`

后处理核心 `path_processing_core` 不依赖 ROS，与 `astar_core` 分离；
由已有 `astar_node` 对同一个 inflated grid 快照执行，未新增中间 ROS 节点或第三方库。

| 接口 | 默认 topic | 类型 / 用途 |
| --- | --- | --- |
| Inflated grid | `/plan/inflated_grid` | `nav_msgs/msg/OccupancyGrid`；不二次膨胀 |
| Goal | `/goal_pose` | `geometry_msgs/msg/PoseStamped` |
| Raw path | `/plan/raw_path` | `nav_msgs/msg/Path`；原始 A* cell 中心序列，仅调试 |
| Processed path | `/plan` | `nav_msgs/msg/Path`；保持 plan -> controller 接口 |
| Status | `/plan/status` | `std_msgs/msg/String`；沿用 Phase 3 六类状态 |

Start 仍为最新的 `planning_frame -> base_footprint` TF，不由 Task 提供。
实际地图 frame/localization 仍未确认；必须显式设置真实 `planning_frame` 并已有对应 TF 链。
Raw/processed Path 具有相同 frame、stamp 和端点坐标，PoseStamped header 与各自 Path 相同。
Path 均为 reliable/volatile/depth 1，RViz/调试订阅应先于目标发送。
Topic 可通过参数/remap 覆盖，所有输入/输出必须在 remap 后互不重名。

## 几何规则

- 去掉连续重复点和同方向共线中间点；不删除折返或真正转角，不使用大距离容差简化。
- Shortcut 为有界 greedy：从当前点尝试最远可见的后续顶点；关闭 shortcut 时仍去重、
  去共线、重采样并计算 yaw，不改变折线的转弯拓扑。
- LOS 使用连续线段在 grid 局部坐标中的保守 supercover 遍历，检查每个栅格边界事件及
  中间区间，不使用稀疏等距采样替代碰撞检查。擦过 blocked cell 的边/角也算碰撞，
  沿 grid line 时检查两侧；地图外视为 blocked。支持 origin 平移、yaw 和任意分辨率。
- `100` blocked，`0` 和 Phase 2 已允许的 `-1` 可通行；不重新解释 unknown 策略。
  检查原始、共线合并、shortcut 和最终重采样路径的线段。
- 沿整条折线累计弧长每 `resample_spacing` 米取样，不在拐点重置采样进度。
  **另外保留每个拐点和终点**：这些局部间距可能小于设定值，但不大于设定值
  （除浮点 roundoff）。这是为避免相邻采样点的连线切进障碍，不是曲线平滑。
- 保留原始 A* 首尾 cell 中心，未追加到精确 TF/goal 坐标的未验证连接段。
- Processed yaw 为当前点到下一点的 `atan2`；终点继承最后一段 yaw。
  单点路径没有切线，使用 yaw=0。Raw path 保留 Phase 3 朝向规则（终点为 goal yaw），
  但 processed path 不强行添加终点原地旋转或速度规划。
- 参数非法时启动失败；输出点数超限或后处理碰撞验证失败时返回 `NO_PATH`、打印原因，
  同时清空两条 Path，不静默发送 raw path 给 controller。
  新地图也清空两条旧 Path，不自动重规划；状态仍描述最近一次请求结果。

## YAML 参数

`src/plan/config/astar.yaml`，参数为只读启动配置，改动后重启节点或使用启动参数覆盖：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `raw_path_topic` | `/plan/raw_path` | 调试输出；与控制器接口分开 |
| `post_processing.enable_shortcut` | `true` | 是否尝试 LOS shortcut |
| `post_processing.resample_spacing` | `0.1` | 米，必须有限且大于零 |
| `post_processing.shortcut_max_lookahead` | `200` | 最多前看多少个简化后顶点，必须大于零 |
| `post_processing.max_output_points` | `100000` | 含拐点/终点的输出上限，防止过小间距造成内存膨胀 |

## Build / Test

在仓库根目录执行，构建、日志和测试输出留在仓库内：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/phase4_path_processing build --base-paths src \
  --build-base build/phase4_path_processing --install-base install/phase4_path_processing \
  --packages-select plan
source install/phase4_path_processing/local_setup.bash
mkdir -p log/phase4_path_processing/ros
export ROS_LOG_DIR="$PWD/log/phase4_path_processing/ros"
colcon --log-base log/phase4_path_processing test \
  --build-base build/phase4_path_processing --install-base install/phase4_path_processing \
  --packages-select plan
colcon test-result --test-result-base build/phase4_path_processing --verbose
```

核心测试覆盖去重、共线/折返、空/单点、shortcut 开关/前看限制、薄障碍、角点、grid line、
unknown、旋转 origin、累计弧长采样、拐点保留、yaw、无效输入及输出上限。
随机 LOS 与独立的 closed-box 解析线段碰撞检测对照；随机 A* 路径验证后处理不碰撞，
且不增加折线长度。ROS 测试验证双 Path、header/端点一致、参数覆盖、yaw、间距、
Phase 2 链路、失败/地图更新清空双输出；原有 Phase 2/3 测试继续运行。
XML lint 需要访问 ROS 官方 schema；cppcheck 2.7 可能被 ament 自动跳过，应检查测试汇总。

## RViz / 调试

每个终端先 source 环境。已有真实地图及定位时，分别启动（替换实际地图 frame）：

```bash
ros2 launch plan planning_grid.launch.py
ros2 launch plan astar.launch.py planning_frame:="实际地图frame"
rviz2 -d src/plan/config/planning_grid.rviz
```

RViz Fixed Frame 设为实际地图 frame，先打开 Path displays，再用 2D Goal Pose 发目标。
橙色为 Raw AStar Path，绿色为 Processed Path；两者端点一致，绿色线段应不跨膨胀障碍、
不擦障碍角点。绿色折线并非 spline，不应据此声称曲率连续或已具备可跟踪速度轨迹。
可分别在另外的终端查看：

```bash
ros2 topic echo /plan/raw_path --once
ros2 topic echo /plan --once
ros2 topic echo /plan/status --qos-durability transient_local
ros2 param get /planner post_processing.resample_spacing
```

无实际定位时可沿用 [Phase 3 独立合成验证](astar_phase3.md#独立合成验证不代表实际-localization)
中的 `grid_debug` 地图和静态测试 TF。必须隔离 ROS domain，不能将假 TF 当作实际定位。
要比较关闭 shortcut 的效果，停止原 A* 节点，使用相同 YAML 在合成环境重启：

```bash
ros2 run plan astar_node --ros-args --params-file src/plan/config/astar.yaml \
  -p planning_frame:=grid_debug -p use_sim_time:=false \
  -p post_processing.enable_shortcut:=false \
  -p post_processing.resample_spacing:=0.2
```

重新发送目标；共线点仍删除，重采样仍执行，但不会跨原折线拐点做 shortcut。
本阶段未引入 spline、优化器、复杂速度规划、LQR 或第三方规划库。
