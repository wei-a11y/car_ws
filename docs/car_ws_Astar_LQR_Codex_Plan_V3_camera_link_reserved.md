# car_ws 第一版 A\* + LQR 修改计划 V3

## 适配当前 Gazebo：2D LiDAR 已仿真，深度相机仅预留 URDF Link

> 用途：直接作为 Codex 分阶段执行计划\
> 主仓库：`wei-a11y/car_ws`\
> 开发分支：`urdf_change`\
> 参考仓库：`wei-a11y/xiache_projector`，参考 `production` 分支\
> 环境：Ubuntu 22.04 + ROS 2 Humble\
> 原则：先做最小可运行导航 baseline，再逐步加入
> Task、深度相机和环境交互。\
> 核心要求：代码量小、模块解耦、便于人工阅读和后续论文/比赛实验。

------------------------------------------------------------------------

# 1. 当前状态与本计划前提

## 1.1 当前 Gazebo

已知：

-   差速移动底盘已经能够在 Gazebo 中运行；
-   键盘遥控已经能够驱动车辆；
-   已添加单线 2D LiDAR Gazebo 仿真插件；
-   URDF 中已经预留深度相机的 `link`；
-   当前 **没有添加深度相机 Gazebo sensor/plugin**；
-   当前阶段不要求 RGB、Depth、CameraInfo、PointCloud2 仿真输出。

因此当前仿真应理解为：

``` text
robot
├── base / wheels
├── 2D LiDAR link
│   └── Gazebo LiDAR plugin      [当前启用]
└── depth camera link            [当前只保留结构和TF]
    └── Gazebo depth plugin      [当前不添加]
```

## 1.2 当前实车

实车计划使用：

``` text
2D LiDAR
+
Depth Camera
```

当前预算不依赖 MID360。

## 1.3 深度相机 Link 的处理原则

当前已有的深度相机 link：

-   保留；
-   Phase 0 检查名称、父 link、joint、origin、TF；
-   如果没有明显 URDF 错误，不修改；
-   不删除；
-   不重复创建新 camera link；
-   不在导航 baseline 阶段添加 Gazebo depth plugin；
-   不提前添加 OpenCV/PCL/YOLO 等依赖；
-   后期直接基于这个 link 添加 Gazebo 深度相机插件。

------------------------------------------------------------------------

# 2. 第一版目标

第一版只完成：

``` text
Goal
 ↓
Task / Goal Interface
 ↓
Planner
 ↓
2D OccupancyGrid
 ↓
Inflation
 ↓
A*
 ↓
Path Post-processing
 ↓
nav_msgs/Path
 ↓
LQR Controller
 ↓
/cmd_vel_raw
 ↓
2D LiDAR Safety Gate
 ↓
/cmd_vel
 ↓
Differential Drive
```

达到：

1.  Gazebo 中指定目标点；
2.  A\* 能规划路径；
3.  RViz 能显示路径；
4.  LQR 能跟踪路径；
5.  到达目标后可靠停车；
6.  路径被阻塞时 Planner 能明确返回 `NO_PATH`；
7.  LaserScan 前方突然出现障碍时能够停车；
8.  不需要深度相机即可完成整个 baseline。

------------------------------------------------------------------------

# 3. 长期架构

后续研究方向保持：

``` text
A* SUCCESS
 ↓
LQR
 ↓
正常导航
```

或：

``` text
A* NO_PATH
 ↓
Planning Failure Analyzer
 ↓
Depth Camera
 ↓
识别/定位阻塞物
 ↓
判断是否可移动
 ↓
Manipulation Planner
 ↓
机械臂推/移障碍物
 ↓
更新环境
 ↓
A* Replan
 ↓
继续导航
```

因此 `NO_PATH` 不是普通错误，而是后续环境交互系统的重要触发接口。

------------------------------------------------------------------------

# 4. 软件边界

现有包优先保持：

``` text
task
plan
controller
common_msg
```

建议职责：

``` text
task
└── 任务状态与目标管理

plan
├── map/grid adapter
├── inflation
├── A*
├── path post-processing
└── planning result

controller
├── LQR path tracker
└── safety command gate
    （若现有结构明显不合适，再考虑独立小包）

common_msg
└── 仅放确实无法用ROS2标准消息表达的消息
```

不要为了"架构漂亮"拆出大量 package。

------------------------------------------------------------------------

# 5. 第一版统一 ROS 接口

优先使用标准消息。

``` text
Goal:
geometry_msgs/PoseStamped

Map:
nav_msgs/OccupancyGrid

Path:
nav_msgs/Path

Robot State:
TF + nav_msgs/Odometry

2D LiDAR:
sensor_msgs/LaserScan

Controller Command:
geometry_msgs/Twist
```

建议：

``` text
controller -> /cmd_vel_raw
safety gate -> /cmd_vel
```

## 明确禁止

不要创建自定义 `Path.msg`。

Task 第一版也不要发送 `start_pose`：

``` text
Task -> goal
Planner -> 从 TF/localization 获取当前 pose
```

这样重新规划时 Planner 始终使用真实当前位置。

------------------------------------------------------------------------

# 6. Phase 0：只读仓库与仿真审计

## 目标

在任何算法代码修改前，把 `urdf_change` 当前实际接口确认清楚。

## 必查内容

### A. Workspace

确认：

-   package 列表；
-   `task`；
-   `plan`；
-   `controller`；
-   `common_msg`；
-   各包当前已有源码；
-   当前依赖关系。

### B. Gazebo

确认：

-   Gazebo 主 launch；
-   world；
-   robot spawn；
-   差速底盘插件/ros2_control；
-   `/cmd_vel`；
-   `/odom`；
-   键盘遥控完整数据流。

### C. TF

列出实际 TF tree，重点：

``` text
map
odom
base_footprint
base_link
lidar_link
depth_camera_link
```

名称必须以仓库实际内容为准。

### D. 2D LiDAR

确认：

-   LiDAR link；
-   Gazebo sensor/plugin；
-   LaserScan topic；
-   frame_id；
-   update rate；
-   min/max range；
-   scan angle；
-   LiDAR 到 base 的 TF。

### E. 深度相机预留 Link

只检查：

-   实际 link 名称；
-   parent link；
-   joint 类型；
-   xyz；
-   rpy；
-   TF 是否合理；
-   是否存在重复 frame。

**不要添加 plugin。**

### F. 地图与定位

确认当前是否已经存在：

``` text
/map
map -> odom
odom -> base_link
```

确认是否已有：

-   map_server；
-   SLAM；
-   localization；
-   Gazebo ground truth；
-   其他 pose source。

## Phase 0 Codex Prompt

``` text
请只读分析 wei-a11y/car_ws 的 urdf_change 分支。
暂时禁止修改任何文件。

当前已知状态：
1. Gazebo 差速底盘已经能够键盘控制；
2. 已添加单线2D LiDAR Gazebo仿真插件；
3. URDF已经预留深度相机link；
4. 当前没有添加深度相机Gazebo sensor/plugin；
5. 当前阶段不要添加深度相机plugin。

最终第一版数据流：
Goal -> A* -> Path -> LQR -> Safety Gate -> cmd_vel -> 差速底盘。

请输出：

A. workspace/package结构；
B. task、plan、controller、common_msg当前内容；
C. Gazebo启动入口；
D. 键盘控制到底盘运动的完整数据流；
E. /cmd_vel、/odom及实际topic名称；
F. 完整TF链；
G. diff-drive实现方式；
H. 可从代码确认的轮距、轮半径等底盘参数；
I. 2D LiDAR link、plugin、LaserScan topic、frame_id和TF；
J. 已预留深度相机link的名称、parent、joint、xyz/rpy和TF；
K. 当前是否存在/map、SLAM、localization、map->odom；
L. 第一版A*+LQR最小修改文件清单。

深度相机link只检查，不修改，不删除，不增加plugin。
无法确认的内容明确写“待确认”，不要猜。

禁止写A*。
禁止写LQR。
禁止大规模重构。
```

## 模型

`GPT-5.6 Terra / Medium`

## 验收

Codex 必须先输出真实接口表。

如果 `/map`、定位或 TF
前置条件不满足，先记录问题，不允许直接猜一个接口继续写算法。

------------------------------------------------------------------------

# 7. Phase 1：冻结接口与配置框架

## 目标

把 Planner 与 Controller 的接口固定下来。

## 完成内容

确定：

``` text
goal topic
map topic
scan topic
odom topic
path topic
cmd_vel_raw
cmd_vel
planning status
```

增加最小 YAML 配置结构，但不要写算法。

建议：

``` text
config/
├── planner.yaml
└── controller.yaml
```

实际路径以当前 package 结构为准。

## 深度相机

此阶段：

``` text
camera link：保留
camera plugin：不添加
camera topic：不要求
camera dependency：不添加
```

## Codex Prompt

``` text
基于Phase 0真实审计结果，完成第一版ROS接口冻结。

要求：
1. goal使用geometry_msgs/PoseStamped；
2. map使用nav_msgs/OccupancyGrid；
3. path使用nav_msgs/Path；
4. state优先使用TF，必要时读取nav_msgs/Odometry；
5. LiDAR使用sensor_msgs/LaserScan；
6. controller输出/cmd_vel_raw；
7. safety gate最终输出/cmd_vel；
8. 不创建自定义Path.msg；
9. Task不发送start_pose；
10. 硬件topic差异优先通过参数/remap解决。

当前深度相机只有URDF link。
不要添加Gazebo camera plugin。
不要添加RGB/depth/PointCloud2代码。
不要修改深度相机link，除非Phase 0确认存在明确URDF错误。

只完成必要接口、配置、package依赖调整。
不要实现A*和LQR。

最后输出：
- 修改文件；
- topic/interface表；
- 数据流；
- colcon build命令；
- 验证命令。
```

## 模型

`GPT-5.6 Terra / Low-Medium`

------------------------------------------------------------------------

# 8. Phase 2：地图、坐标系与 Inflation

## 目标

得到 A\* 可以直接使用的 2D planning grid。

``` text
nav_msgs/OccupancyGrid
 ↓
Grid Adapter
 ↓
Raw Grid
 ↓
Inflation
 ↓
Inflated Planning Grid
```

## 关键前置

A\* 必须保证：

``` text
start
goal
map
```

处于同一坐标系。

### 如果 Phase 0 已有完整定位

直接接入。

### 如果没有 `map -> odom`

不要让 Codex私自实现一套复杂定位。

应先输出当前缺失项，再采用本项目确定的最小仿真定位方案。

## Inflation

第一版实现简单障碍膨胀。

参数化：

``` yaml
occupied_threshold: 65
unknown_is_obstacle: true
robot_radius: ...
safety_margin: ...
inflation_radius: ...
```

机器人尺寸从 URDF/实际尺寸确认，不允许 Codex 猜。

保留：

``` text
raw grid
inflated grid
```

未来用于分析：

``` text
原始几何可通行
→ inflation后不可通行
```

## Codex Prompt

``` text
在plan包中实现第一版2D planning grid。

输入：
nav_msgs/OccupancyGrid

要求：
1. OccupancyGrid转内部grid；
2. occupied threshold参数化；
3. unknown cell策略参数化；
4. 基于机器人尺寸+safety margin进行障碍膨胀；
5. 同时保留raw grid与inflated grid；
6. 参数写YAML；
7. 核心grid/inflation逻辑与ROS node尽量分离；
8. 提供RViz或调试验证方式。

机器人尺寸必须从Phase 0确认结果获得，不允许猜。

不要实现：
A*
LQR
dynamic obstacle
depth camera
PointCloud2
Nav2完整costmap。
```

## 模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 9. Phase 3：A\* 核心

## 目标

完成最小、可测试的 2D A\*。

## 第一版

建议：

-   8-connected；
-   直线代价 1；
-   对角代价 sqrt(2)；
-   octile/Euclidean heuristic；
-   world ↔ grid；
-   parent 回溯；
-   boundary check；
-   collision check；
-   明确 corner-cutting 策略。

## Planner Result

至少区分：

``` text
SUCCESS
INVALID_START
INVALID_GOAL
NO_PATH
MAP_UNAVAILABLE
TF_UNAVAILABLE
```

状态接口实现形式根据 Phase 0/1 现有结构选择最轻量方案。

## 结构

尽量：

``` text
AStarPlanner
    ↑
pure/testable algorithm

PlannerNode
    ↑
ROS wrapper
```

不要写成一个巨型 ROS callback。

## Codex Prompt

``` text
在plan包实现最小2D A*。

要求：
1. 使用Phase 2 inflated grid；
2. 8邻域；
3. 正确处理直线/对角代价；
4. 明确并实现corner-cutting策略；
5. start来自当前TF/localization；
6. goal来自geometry_msgs/PoseStamped；
7. 输出nav_msgs/Path；
8. Path frame与planning map一致；
9. 返回SUCCESS、INVALID_START、INVALID_GOAL、NO_PATH、MAP_UNAVAILABLE、TF_UNAVAILABLE；
10. A*核心与ROS Node尽量分离；
11. 增加最小单元测试。

禁止：
Hybrid A*
Nav2 Planner Plugin重构
动态规划
机械臂逻辑
深度相机逻辑。

先列修改文件，再实施。
最后给出build、test和RViz验证步骤。
```

## 模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 10. Phase 4：Path Post-processing

## 原因

栅格 A\* 路径不应直接作为最终 LQR 参考路径。

## 流程

``` text
A* raw path
 ↓
remove duplicate/collinear points
 ↓
collision-checked line-of-sight shortcut
 ↓
fixed-distance resample
 ↓
compute yaw
 ↓
nav_msgs/Path
```

第一版不要 spline。

## 安全要求

任何 shortcut 都必须在 **inflated grid** 上进行碰撞检查。

## Codex Prompt

``` text
给A*增加轻量path post-processing。

只实现：
1. 删除重复/共线点；
2. 可配置line-of-sight shortcut；
3. shortcut必须在inflated grid上碰撞检查；
4. 固定间距重采样；
5. 根据相邻路径点计算yaw；
6. 输出nav_msgs/Path。

不要使用：
spline
优化器
复杂速度规划
第三方规划库。

增加开关，使raw path和processed path都能用于调试/RViz比较。
```

## 模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 11. Phase 5：LQR 数学模型审查

这一阶段 **先不写大量代码**。

## 目标

根据实际差速底盘确定第一版 LQR 模型。

至少明确：

-   state/error definition；
-   `e_y`；
-   `e_yaw`；
-   是否需要 `e_x`；
-   control input；
-   reference linear velocity；
-   discrete dt；
-   A/B matrix；
-   Q/R；
-   nearest point；
-   lookahead；
-   terminal behavior。

第一版优先简单、低速、可解释。

## Codex Prompt

``` text
暂时不要直接写LQR代码。

基于当前差速底盘、现有odom/TF接口和nav_msgs/Path，
先设计第一版低速LQR path tracker数学模型。

目标：
代码量小、可解释、适合Gazebo和后续实车。

请输出：
1. 差速车运动学模型；
2. 路径跟踪误差定义；
3. 状态向量；
4. 控制输入；
5. 连续/离散模型；
6. A/B矩阵如何得到；
7. Q/R含义；
8. v_ref处理；
9. nearest/lookahead策略；
10. 终点减速和停车策略；
11. 需要从现有仓库读取的参数；
12. 推荐最小实现结构。

不要复制xiache_projector的复杂工业控制逻辑。
不要加入MPC、jerk规划、换挡、brake hold。
```

## 模型

`GPT-5.6 Sol / High`

## 验收

人工确认数学模型后再进入 Phase 6。

------------------------------------------------------------------------

# 12. Phase 6：LQR 实现

## 输入

``` text
nav_msgs/Path
TF/Odometry
```

## 输出

``` text
geometry_msgs/Twist
/cmd_vel_raw
```

## 参数

至少参数化：

``` text
control_frequency
v_ref
max_v
max_w
Q
R
lookahead
goal_position_tolerance
goal_yaw_tolerance
slowdown_distance
```

## 日志

至少能记录：

``` text
x
y
yaw
target_index
e_y
e_yaw
v_cmd
w_cmd
goal_distance
controller_state
```

## Codex Prompt

``` text
按照已经确认的Phase 5数学模型实现LQR path tracker。

输入：
nav_msgs/Path
TF/Odometry

输出：
geometry_msgs/Twist -> /cmd_vel_raw

要求：
1. LQR核心与ROS Node尽量分离；
2. nearest/lookahead；
3. lateral error；
4. heading error；
5. angle normalization；
6. Q/R参数化；
7. v/w限幅；
8. 接近终点减速；
9. 到达目标可靠输出0；
10. path丢失/TF异常时输出安全0速度；
11. 输出必要调试日志。

不要添加：
MPC
复杂纵向规划
jerk状态机
换挡
brake hold
深度相机。

给出build、运行和调参方法。
```

## 模型

`GPT-5.6 Sol / Medium-High`

------------------------------------------------------------------------

# 13. Phase 7：2D LiDAR Safety Gate

## 原因

A\* + LQR 不等于动态避障。

实车上如果有人/箱子突然进入车辆前方，不能等待重新全局规划才停车。

第一版只实现非常轻量的安全停车，不引入 DWA/TEB。

## 数据流

``` text
LQR
 ↓
/cmd_vel_raw
 ↓
Safety Gate ← /scan
 ↓
/cmd_vel
 ↓
base
```

## 功能

-   前向扇区；
-   最小距离；
-   stop distance；
-   release distance；
-   hysteresis；
-   LaserScan timeout；
-   无有效 scan 时自动模式停止。

## Codex Prompt

``` text
实现独立的2D LiDAR safety command gate。

输入：
/cmd_vel_raw
sensor_msgs/LaserScan

输出：
/cmd_vel

要求：
1. 前向扇区角度参数化；
2. stop_distance参数化；
3. release_distance参数化；
4. 简单hysteresis，避免阈值附近抖动；
5. LaserScan超时或数据无效时自动模式输出0；
6. safety逻辑与LQR解耦；
7. 提供enable参数，方便仿真调试。

不要实现：
DWA
TEB
局部绕障
深度相机安全层。
```

## 模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 14. Phase 8：单目标 Gazebo 闭环

完整链路：

``` text
Goal
 ↓
Map + Inflation
 ↓
A*
 ↓
Path Processing
 ↓
LQR
 ↓
Safety Gate
 ↓
Gazebo Robot
```

## 测试场景

至少：

1.  直线；
2.  单个 90° 转弯；
3.  多拐点；
4.  静态障碍绕行；
5.  窄通道；
6.  invalid goal；
7.  NO_PATH；
8.  LaserScan safety stop。

## 记录指标

``` text
planning time
path length
cross-track RMSE
max cross-track error
heading error
terminal position error
navigation time
success/failure
```

这些数据后续可以直接作为实验 baseline。

## 模型

普通问题：`GPT-5.6 Terra / Medium`

跨 A\*/TF/LQR 的复杂问题：`GPT-5.6 Sol / Medium-High`

------------------------------------------------------------------------

# 15. Phase 9：Task Pickup → Delivery

单目标稳定后再开发 Task。

## 状态机

``` text
IDLE
 ↓
GO_PICKUP
 ↓
PICKUP_REACHED
 ↓
GO_DELIVERY
 ↓
DELIVERY_REACHED
 ↓
DONE
```

第一版：

-   到 pickup 认为取货完成；
-   到 delivery 认为放货完成；
-   不调用机械臂。

Task 只负责：

``` text
state
goal
navigation result
```

禁止把 A\*/LQR 写进 Task。

## 模型

`GPT-5.6 Terra / Low-Medium`

------------------------------------------------------------------------

# 16. Phase 10：Planning Failure 接口

在开始机械臂之前，先把研究接口稳定下来。

## NO_PATH 时保存

``` text
timestamp
start
goal
raw grid
inflated grid
planning result
```

后续可以增加：

``` text
blocking region
clearance
candidate obstacle
```

但第一版不要过度设计。

## 目标

形成：

``` text
Navigation Planner
       │
       ├── SUCCESS -> controller
       │
       └── NO_PATH -> future interaction module
```

## 模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 17. Phase 11：添加 Gazebo 深度相机 Plugin

**只有到这一阶段才添加。**

当前已有 camera link，因此原则是：

``` text
existing depth_camera_link
 ↓
add Gazebo sensor/plugin
 ↓
RGB
Depth
CameraInfo
PointCloud2（需要时）
```

## 首先检查

Phase 0 已确认：

-   link name；
-   parent；
-   origin；
-   optical frame 是否已有；
-   TF。

如果已有结构正确：

**不要重新创建 camera link。**

如果 ROS 相机消息需要 optical frame，可在保持机械安装 link 的基础上按
ROS 相机坐标约定补充必要 optical frame；必须解释原因。

## 第一阶段相机验收

只要求：

``` text
RGB visible
Depth visible
CameraInfo valid
TF correct
RViz visualization correct
```

PointCloud2 根据使用的 Gazebo/ROS 插件能力决定是否直接输出或后续转换。

## 当前阶段仍不做

-   YOLO；
-   segmentation；
-   movable classification；
-   机械臂；
-   完整 3D navigation；
-   MID360。

## Codex Prompt

``` text
现在开始为Gazebo加入深度相机仿真。

重要前提：
URDF中已经存在预留的深度相机link。
禁止无理由重新创建或移动这个link。

请先读取现有URDF/Xacro和Phase 0记录，
确认：
1. camera link；
2. parent；
3. joint；
4. xyz/rpy；
5. TF；
6. 是否已有optical frame。

然后以最小修改方式在现有camera link上增加Gazebo深度相机sensor/plugin。

目标只包括：
- RGB
- Depth
- CameraInfo
- 正确TF
- RViz验证

如果插件可直接输出PointCloud2，可保留；
如果不能，不要为了PointCloud2大规模增加依赖。

不要实现：
YOLO
目标识别
机械臂控制
3D SLAM
MID360。

最后给出topic列表、frame关系、启动命令和RViz验证步骤。
```

## 模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 18. Phase 12：深度相机障碍补盲

导航 baseline 和相机仿真都稳定后再做。

目的：

补充单线 LiDAR 容易漏掉的：

-   低矮障碍；
-   高于扫描面的障碍；
-   悬空结构。

第一版可以：

``` text
Depth
 ↓
PointCloud / Depth ROI
 ↓
height filtering
 ↓
2D projection
 ↓
supplemental obstacle grid
```

然后：

``` text
2D LiDAR obstacle
+
Depth obstacle
 ↓
Planning Grid
```

但必须保持：

**深度相机失效时，2D LiDAR baseline 仍然能够独立运行。**

## 模型

`GPT-5.6 Sol / Medium`

------------------------------------------------------------------------

# 19. Phase 13：环境交互最小 Demo

目标场景：

``` text
Robot
 ↓
Blocked corridor
 ↓
A*
 ↓
NO_PATH
 ↓
Depth Camera
 ↓
detect known box
 ↓
estimate position
 ↓
movable candidate
 ↓
predefined manipulation primitive
 ↓
box moved
 ↓
update obstacle information
 ↓
A* replan
 ↓
SUCCESS
 ↓
LQR continue
```

第一版优先：

-   已知箱子；
-   少量目标类别；
-   简单推；
-   预定义操作动作；
-   不做通用抓取；
-   不做 RL；
-   不做 VLA；
-   不做完整 affordance learning。

先证明：

> 机器人能够在传统导航规划失败后，通过主动改变环境恢复可通行性。

## 模型

架构/算法：`GPT-5.6 Sol / High`

常规 ROS 接线：`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 20. 当前明确不需要 MID360

当前系统：

``` text
2D LiDAR
+
Depth Camera
+
Wheel Odometry
(+ IMU if available)
```

已经足以完成当前研究路线的第一版验证。

只有实验明确出现以下需求时再重新评估 3D LiDAR：

-   需要 360° 三维障碍覆盖；
-   深度相机视场成为主要瓶颈；
-   3D 建图本身成为研究目标；
-   多层/坡道/复杂非平面环境成为主要任务；
-   当前传感器组合无法满足实验所需可靠性。

不要为了"传感器规格更高"提前引入 MID360。

------------------------------------------------------------------------

# 21. Codex 使用策略

每次只执行一个 Phase。

所有执行 Prompt 末尾追加：

``` text
通用执行约束：

1. 先读取相关文件，再修改；
2. 不修改本阶段无关文件；
3. 不大规模重构现有URDF/Gazebo；
4. 优先ROS2标准消息；
5. 不创建自定义Path.msg；
6. 参数放YAML，避免magic number；
7. 核心算法尽量与ROS wrapper分离；
8. 参考xiache_projector时只借鉴成熟设计思路，不复制其复杂业务逻辑；
9. 输出修改文件列表；
10. 输出修改前后数据流；
11. 输出colcon build命令；
12. 输出运行/测试命令；
13. 输出验收标准；
14. 如果发现关键前置条件不满足，停止扩大修改范围并报告；
15. 不自行增加本Phase未要求的功能；
16. 每个Phase完成后单独git commit。
```

------------------------------------------------------------------------

# 22. 模型选择

为了控制 Codex 消耗：

  工作                        推荐
  --------------------------- --------------------------
  读仓库、简单 ROS 接口       GPT-5.6 Terra Low/Medium
  CMake/package/launch 小改   GPT-5.6 Terra Low
  A\* / Inflation             GPT-5.6 Terra Medium
  路径后处理                  GPT-5.6 Terra Medium
  LQR 数学设计                GPT-5.6 Sol High
  LQR 实现/复杂调试           GPT-5.6 Sol Medium-High
  TF/跨模块疑难 Bug           GPT-5.6 Sol Medium-High
  深度相机 plugin 接线        GPT-5.6 Terra Medium
  环境交互总体架构            GPT-5.6 Sol High

原则：

> Terra 完成大多数工程修改；Sol 留给数学、架构和真正复杂的跨模块问题。

------------------------------------------------------------------------

# 23. 推荐执行顺序

``` text
Phase 0
只读审计
 ↓
Phase 1
冻结接口
 ↓
Phase 2
Map + TF + Inflation
 ↓
Phase 3
A*
 ↓
Phase 4
Path Post-processing
 ↓
Phase 5
LQR数学审查
 ↓
Phase 6
LQR实现
 ↓
Phase 7
2D LiDAR Safety Gate
 ↓
Phase 8
单目标Gazebo闭环
 ↓
Phase 9
Pickup/Delivery Task
 ↓
Phase 10
Planning Failure接口
 ↓
──────── 导航 baseline 完成 ────────
 ↓
Phase 11
在现有Camera Link上添加Gazebo Depth Plugin
 ↓
Phase 12
Depth障碍补盲
 ↓
Phase 13
Environment Interaction Demo
```

------------------------------------------------------------------------

# 24. 当前立即执行的任务

现在只执行 **Phase 0**。

不要：

-   添加深度相机 plugin；
-   写 A\*；
-   写 LQR；
-   修改机械臂；
-   引入 Nav2 完整栈；
-   引入 MID360；
-   提前写 PointCloud/YOLO。

Phase 0 完成后，根据 Codex 输出的真实：

``` text
TF
/cmd_vel
/odom
/scan
/map
package结构
camera预留link
```

再决定 Phase 1 的具体文件级修改。

这样可以最大程度避免 Codex 根据假设修改错误文件，也最节省额度。
