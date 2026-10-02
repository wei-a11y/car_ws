# car_ws A\* + LQR 修改计划 V4

## 先建立 AGENTS.md，再完成 2D LiDAR 导航 Baseline，最后接入深度相机与环境交互

> **用途**：直接作为 Codex 分阶段执行计划\
> **主仓库**：`wei-a11y/car_ws`\
> **当前开发分支**：`urdf_change`\
> **参考仓库**：`wei-a11y/xiache_projector`，主要参考 `production` 分支\
> **环境**：Ubuntu 22.04 + ROS 2 Humble\
> **当前目标**：以最小代码量完成
> `A* + Path Processing + LQR + 2D LiDAR Safety Gate` 导航 baseline。\
> **长期目标**：在导航失败时利用深度相机识别/定位可移动障碍物，通过机械臂改变环境后重新规划。\
> **开发原则**：先跑通、再验证、再扩展；模块解耦；优先 ROS 2
> 标准接口；控制 Codex 修改范围和额度消耗。

------------------------------------------------------------------------

# 0. 当前已知状态

## 0.1 Gazebo

当前已经明确：

``` text
差速底盘                         已有
键盘遥控                         已能驱动车辆
2D LiDAR Link                    已有
2D LiDAR Gazebo Plugin           已有
深度相机 Link                     URDF 中已预留
深度相机 Gazebo Sensor/Plugin     当前没有
```

因此当前机器人结构应理解为：

``` text
robot
├── base / wheels
├── 2D LiDAR link
│   └── Gazebo LiDAR plugin        [当前启用]
└── depth camera link              [当前只保留结构/TF]
    └── depth camera plugin        [后续添加]
```

## 0.2 当前实车传感器路线

计划使用：

``` text
2D LiDAR
+
Depth Camera
+
Wheel Odometry
(+ IMU，如果当前硬件已有)
```

当前方案 **不依赖 MID360**。

## 0.3 当前深度相机 Link 规则

现有深度相机 link：

-   保留；
-   不删除；
-   不重复创建；
-   Step -1 只把"已预留 camera link"写入项目规则；
-   Phase 0 检查实际名称、parent、joint、xyz/rpy 和 TF；
-   导航 baseline 阶段不添加 Gazebo depth plugin；
-   不提前增加 RGB/Depth/PointCloud2/PCL/OpenCV/YOLO 相关代码；
-   后续直接在现有 link 基础上添加 Gazebo 深度相机仿真。

------------------------------------------------------------------------

# 1. V4 总执行顺序

``` text
Step -1
生成根目录 AGENTS.md V1
       ↓
Phase 0
只读审计真实仓库
       ↓
Step 0.5
根据 Phase 0 更新 AGENTS.md V2
       ↓
Phase 1
冻结 ROS 接口
       ↓
Phase 2
Map / TF / Inflation
       ↓
Phase 3
A*
       ↓
Phase 4
Path Post-processing
       ↓
Phase 5
LQR 数学模型审查
       ↓
Phase 6
LQR 实现
       ↓
Phase 7
2D LiDAR Safety Gate
       ↓
Phase 8
单目标 Gazebo 闭环
       ↓
Phase 9
Pickup → Delivery Task
       ↓
Phase 10
Planning Failure / NO_PATH 接口
       ↓
======== 导航 Baseline 完成 ========
       ↓
Phase 11
在现有 Camera Link 上添加 Gazebo Depth Plugin
       ↓
Phase 12
Depth 障碍补盲
       ↓
Phase 13
Environment Interaction Demo
```

------------------------------------------------------------------------

# 2. Step -1：生成根目录 AGENTS.md V1

## 2.1 为什么先生成

`AGENTS.md` 用于固定长期开发规则。

它不是 Roadmap，不负责描述每个 Phase 的全部实现步骤。

职责划分：

``` text
AGENTS.md
└── 长期规则 + 已确认的仓库事实

V4 修改计划
└── 阶段目标 + Codex Prompt + 验收标准

Phase Prompt
└── 本次实际任务
```

这样可以避免每个 Codex Prompt 重复大量背景，也减少 Codex
自行扩大修改范围。

------------------------------------------------------------------------

## 2.2 AGENTS.md V1 只能写"已经确定的事实"

V1 可以写：

-   ROS 2 Humble；
-   Ubuntu 22.04；
-   当前开发分支 `urdf_change`；
-   第一版目标是轻量 `A* + LQR`；
-   已有差速 Gazebo 底盘；
-   已有 2D LiDAR Gazebo plugin；
-   URDF 已预留深度相机 link；
-   当前没有深度相机 Gazebo plugin；
-   当前主要 package 为 `task / plan / controller / common_msg`；
-   优先 ROS 2 标准消息；
-   `plan -> controller` 使用 `nav_msgs/Path`；
-   Task 不负责提供机器人实时 start pose；
-   核心算法与 ROS wrapper 尽量分离；
-   参数优先放 YAML；
-   不随意重构 URDF/Gazebo；
-   每次只完成当前 Phase；
-   修改前先读代码；
-   修改后必须编译/测试；
-   不确定的接口必须检查，不允许猜。

------------------------------------------------------------------------

## 2.3 AGENTS.md V1 禁止写死

Phase 0 尚未确认的内容不能作为事实写入：

``` text
/scan 的实际 topic
/odom 的实际 topic
/cmd_vel 的实际 topic
base frame 实际名称
odom frame 实际名称
map frame 实际名称
深度相机 link 实际名称
LiDAR link 实际名称
轮距
轮半径
Gazebo plugin 具体类型
map/localization 实现
launch 文件实际路径
```

如果需要提到，只能写：

> "由 Phase 0 从仓库实际代码确认，禁止假设。"

------------------------------------------------------------------------

## 2.4 AGENTS.md V1 推荐内容结构

``` text
# Project Scope

# Confirmed Environment

# Current Robot State

# Package Responsibilities

# ROS Interface Principles

# Architecture Rules

# URDF / Gazebo Rules

# Sensor Rules

# Coding Rules

# Build and Test Rules

# Codex Working Rules

# Explicit Non-Goals

# Repository Facts To Be Filled After Phase 0
```

------------------------------------------------------------------------

## 2.5 Step -1 Codex Prompt

``` text
任务：在 car_ws 仓库根目录创建 AGENTS.md V1。

当前开发分支：urdf_change。

注意：
本次只创建/修改根目录 AGENTS.md。
不要修改任何源码、URDF、Xacro、launch、CMakeLists.txt、package.xml 或配置文件。

AGENTS.md 的定位：
它是 Codex 后续工作的长期项目规则，不是 Roadmap，不要把整个修改计划复制进去。

已确认事实：
1. 环境为 Ubuntu 22.04 + ROS 2 Humble；
2. 当前开发分支为 urdf_change；
3. 第一版目标是轻量自研：
   Goal -> A* -> Path Processing -> LQR -> Safety Gate -> 差速底盘；
4. Gazebo 差速底盘已经能够通过键盘遥控运动；
5. 已存在单线 2D LiDAR Gazebo 仿真插件；
6. URDF 中已经预留深度相机 link；
7. 当前没有添加深度相机 Gazebo sensor/plugin；
8. 当前主要模块为 task、plan、controller、common_msg；
9. 后续研究方向是：
   navigation failure -> movable obstacle perception -> environment interaction -> replan。

请在 AGENTS.md 中固定以下长期规则：

A. 优先使用 ROS 2 标准消息；
B. plan -> controller 使用 nav_msgs/Path；
C. 禁止无理由创建自定义 Path.msg；
D. Task 不负责发送机器人实时 start pose；
E. 当前 pose 应来自 TF/localization/odometry；
F. 核心算法尽量与 ROS Node wrapper 分离；
G. 参数优先放 YAML，避免 magic number；
H. 不随意大规模重构现有 URDF/Gazebo；
I. 当前保留深度相机 link，但导航 baseline 阶段禁止提前添加 depth plugin；
J. 不提前引入 PCL、YOLO、完整 3D navigation；
K. 不引入 MID360 依赖；
L. 不为了实现 A*+LQR 而直接引入完整 Nav2；
M. 每次只完成当前 Phase；
N. 修改前先读取相关文件；
O. 不修改当前任务无关文件；
P. 不确定的接口必须从仓库确认，禁止猜；
Q. 修改后给出 build/test 命令；
R. 关键前置条件不满足时停止扩大修改范围并报告；
S. 每个 Phase 建议独立 git commit。

特别注意：
当前还没有完成 Phase 0 仓库审计。

因此以下内容禁止在 AGENTS.md V1 中写成确定值：
- /scan
- /odom
- /cmd_vel
- base frame
- odom frame
- map frame
- LiDAR link 名称
- camera link 名称
- wheel radius
- wheel separation
- map/localization 实现
- launch 文件路径

这些项目统一放入：
“Repository Facts To Be Filled After Phase 0”
并注明必须从实际仓库确认。

完成后：
1. 展示 AGENTS.md 完整内容；
2. 解释每个 section 的用途；
3. git diff -- AGENTS.md；
4. 不执行其他修改。
```

## 2.6 推荐模型

``` text
GPT-5.6 Luna / Low
或
GPT-5.6 Terra / Low
```

这是规则文件生成，不需要使用 Sol。

## 2.7 验收

必须满足：

-   仓库根目录存在 `AGENTS.md`；
-   没有修改其他文件；
-   没有猜测未知 topic/frame/尺寸；
-   没有把 V4 Roadmap 全部复制进去；
-   明确当前禁止提前添加 depth plugin；
-   明确后续 Codex 工作必须先遵守 AGENTS.md。

## 2.8 建议 Commit

``` text
docs: add project AGENTS instructions
```

------------------------------------------------------------------------

# 3. Phase 0：按 AGENTS.md 执行只读仓库审计

## 3.1 目标

在写任何算法前确认 `urdf_change` 的真实数据流。

**Phase 0 不修改代码。**

Codex 开始前首先读取：

``` text
car_ws/AGENTS.md
```

------------------------------------------------------------------------

## 3.2 必查内容

### Workspace

确认：

-   package 列表；
-   `task`；
-   `plan`；
-   `controller`；
-   `common_msg`；
-   当前源码；
-   package 依赖。

### Gazebo

确认：

-   主 launch；
-   world；
-   robot spawn；
-   diff-drive plugin 或 ros2_control；
-   keyboard teleop → robot 的完整数据流。

### Topic

确认实际：

``` text
cmd_vel
odom
scan
```

不要根据常见命名猜。

### TF

确认实际 TF tree，重点寻找：

``` text
map
odom
base_footprint/base_link
LiDAR frame
camera reserved frame
```

### 2D LiDAR

确认：

-   link；
-   joint；
-   Gazebo plugin；
-   LaserScan topic；
-   frame_id；
-   scan range；
-   angle；
-   update rate；
-   LiDAR → base TF。

### 深度相机预留结构

只检查：

-   link 实际名称；
-   parent；
-   joint；
-   xyz；
-   rpy；
-   TF；
-   是否已有 optical frame；
-   是否存在重复/冲突 frame。

禁止添加 plugin。

### 底盘参数

从实际代码确认：

-   wheel radius；
-   wheel separation；
-   diff-drive 参数；
-   velocity limit（如果存在）。

### 地图与定位

确认是否已经存在：

``` text
/map
map -> odom
localization
SLAM
map_server
Gazebo ground truth
```

------------------------------------------------------------------------

## 3.3 Phase 0 Codex Prompt

``` text
首先读取仓库根目录 AGENTS.md，并严格遵守。

然后只读分析 wei-a11y/car_ws 的 urdf_change 分支。

本 Phase 禁止修改任何文件。

当前第一版目标：
Goal -> A* -> Path Processing -> LQR -> Safety Gate -> cmd_vel -> 差速底盘。

请输出：

1. workspace/package 结构；
2. task、plan、controller、common_msg 当前文件和职责；
3. Gazebo 启动入口；
4. keyboard teleop 到机器人运动的完整数据流；
5. 实际 cmd_vel topic；
6. 实际 odom topic；
7. 实际 LaserScan topic；
8. 完整 TF tree 和关键 frame；
9. diff-drive 实现方式；
10. wheel radius / wheel separation 等可确认参数；
11. 2D LiDAR link、joint、plugin、topic、frame_id、TF；
12. 已预留深度相机 link 的实际名称、parent、joint、xyz/rpy、TF；
13. 是否已经存在 optical frame；
14. 当前是否存在 /map、SLAM、localization、map->odom；
15. task/plan/controller/common_msg 当前哪些内容可以复用；
16. 第一版 A*+LQR 最小修改文件清单；
17. 哪些信息仍然无法从仓库确认。

要求：
- 不修改代码；
- 不添加 depth camera plugin；
- 不写 A*；
- 不写 LQR；
- 不大规模设计未来架构；
- 无法确认的项目写“待确认”，禁止猜。

最后给出一张：
“Confirmed Repository Facts”
表格，供下一阶段更新 AGENTS.md 使用。
```

## 3.4 推荐模型

``` text
GPT-5.6 Terra / Medium
```

## 3.5 验收

得到一份可以直接用于更新 `AGENTS.md` 的真实接口表。

------------------------------------------------------------------------

# 4. Step 0.5：根据 Phase 0 更新 AGENTS.md V2

## 4.1 目标

将 Phase 0 **已经从仓库确认** 的稳定事实写回根目录 `AGENTS.md`。

例如：

``` text
实际 cmd_vel topic
实际 odom topic
实际 scan topic
实际 base frame
实际 odom frame
实际 LiDAR frame
实际 camera reserved link
wheel radius
wheel separation
Gazebo drive implementation
主要 launch 入口
```

只写已经确认且预计长期稳定的信息。

------------------------------------------------------------------------

## 4.2 不应该写入 AGENTS.md 的内容

不要加入：

-   临时调试日志；
-   某次 bug；
-   当前 Phase 的具体 TODO；
-   一次性的测试结果；
-   尚未决定的 Q/R；
-   A\* 临时参数；
-   某次实验坐标；
-   大段 Roadmap。

这些应该留在 Phase 文档/issue/实验记录里。

------------------------------------------------------------------------

## 4.3 Step 0.5 Codex Prompt

``` text
首先读取：
1. 根目录 AGENTS.md V1；
2. Phase 0 的 Confirmed Repository Facts。

本次只允许修改根目录 AGENTS.md。

任务：
将 Phase 0 已经从实际仓库确认、且预计长期稳定的事实补充到 AGENTS.md。

可以补充：
- 实际 cmd_vel topic；
- 实际 odom topic；
- 实际 LaserScan topic；
- 关键 TF frame；
- LiDAR link/frame；
- 已预留 camera link/frame；
- wheel radius；
- wheel separation；
- Gazebo drive implementation；
- 主要 launch 入口；
- 已确认的 map/localization 状态。

规则：
1. 只写 Phase 0 已确认内容；
2. 未确认内容继续标记待确认；
3. 不写当前 Phase TODO；
4. 不复制 Roadmap；
5. 不修改任何其他文件；
6. 保留 AGENTS.md 原有长期工程规则。

最后输出：
- 修改摘要；
- 新增的 confirmed facts；
- 仍未确认的 facts；
- git diff -- AGENTS.md。
```

## 4.4 推荐模型

``` text
GPT-5.6 Luna / Low
```

## 4.5 建议 Commit

``` text
docs: record confirmed repository interfaces
```

------------------------------------------------------------------------

# 5. Phase 1：冻结第一版 ROS 接口

## 5.1 目标

根据真实仓库状态固定第一版模块接口。

优先：

``` text
Goal:
geometry_msgs/PoseStamped

Map:
nav_msgs/OccupancyGrid

Path:
nav_msgs/Path

Robot State:
TF
必要时 nav_msgs/Odometry

LiDAR:
sensor_msgs/LaserScan

Controller Command:
geometry_msgs/Twist
```

建议数据流：

``` text
Task/Goal
   ↓
Planner
   ↓
nav_msgs/Path
   ↓
LQR
   ↓
/cmd_vel_raw
   ↓
Safety Gate
   ↓
现有底盘 cmd_vel topic
```

实际 topic 名称以 `AGENTS.md V2` 为准。

------------------------------------------------------------------------

## 5.2 规则

-   不创建自定义 `Path.msg`；
-   Task 不发送 start pose；
-   Planner 从 TF/localization 获取当前 pose；
-   硬件 topic 差异优先 parameter/remap；
-   深度相机仍然不添加 plugin；
-   不提前添加 PointCloud2 代码。

------------------------------------------------------------------------

## 5.3 Codex Prompt

``` text
首先读取根目录 AGENTS.md。

基于 Phase 0 已确认的真实接口，完成第一版 ROS 接口冻结。

要求：
1. goal 使用 geometry_msgs/PoseStamped；
2. map 使用 nav_msgs/OccupancyGrid；
3. path 使用 nav_msgs/Path；
4. 当前 pose 优先来自 TF/localization；
5. 必要时读取 nav_msgs/Odometry；
6. 2D LiDAR 使用 sensor_msgs/LaserScan；
7. LQR 输出独立 raw command topic；
8. Safety Gate 输出到现有底盘速度命令接口；
9. 不创建自定义 Path.msg；
10. Task 不发送 start_pose；
11. 参数/remap 优先于硬编码 topic。

当前深度相机只有 URDF 预留 link。
禁止添加 Gazebo depth plugin、RGB/depth/PointCloud2 节点或依赖。

只修改本阶段必要文件。
不要实现 A*。
不要实现 LQR。

最后输出：
- 修改文件；
- 最终 interface table； 
- 数据流；
- build 命令；
- 验证命令。
```

## 推荐模型

`GPT-5.6 Terra / Low-Medium`

------------------------------------------------------------------------

# 6. Phase 2：Map、坐标系与 Inflation

## 6.1 数据流

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

## 6.2 坐标要求

A\* 的：

``` text
start
goal
map
```

必须位于统一 frame。

如果 Phase 0 发现缺少 `map -> odom` 或定位：

**不要让 Codex 私自实现复杂定位系统。**

先明确当前最小仿真定位方案，再继续。

------------------------------------------------------------------------

## 6.3 Inflation

参数化：

``` yaml
occupied_threshold: 65
unknown_is_obstacle: true
robot_radius: ...
safety_margin: ...
inflation_radius: ...
```

机器人尺寸从仓库/URDF确认。

禁止猜。

保留：

``` text
raw_grid
inflated_grid
```

这是未来 Planning Failure 分析的重要输入。

------------------------------------------------------------------------

## 6.4 Codex Prompt

``` text
首先读取 AGENTS.md。

在 plan 包中实现第一版 2D planning grid。

输入：
nav_msgs/OccupancyGrid

实现：
1. OccupancyGrid -> 内部 grid；
2. occupied threshold；
3. unknown cell 策略；
4. robot size + safety margin 的障碍膨胀；
5. raw grid 与 inflated grid 都保留；
6. 参数进入 YAML；
7. grid/inflation 核心尽量与 ROS Node 分离；
8. 提供调试/RViz验证方式。

机器人尺寸必须使用已确认值。

不要实现：
A*
LQR
dynamic obstacle
depth camera
PointCloud2
完整 Nav2 costmap。
```

## 推荐模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 7. Phase 3：A\*

## 第一版功能

-   8-connected；
-   正确直线/对角代价；
-   octile 或 Euclidean heuristic；
-   world ↔ grid；
-   boundary check；
-   collision check；
-   parent 回溯；
-   明确 corner-cutting 策略。

输出：

``` text
nav_msgs/Path
```

必须区分：

``` text
SUCCESS
INVALID_START
INVALID_GOAL
NO_PATH
MAP_UNAVAILABLE
TF_UNAVAILABLE
```

------------------------------------------------------------------------

## Codex Prompt

``` text
首先读取 AGENTS.md。

在 plan 包中实现最小 2D A*。

要求：
1. 使用 Phase 2 inflated grid；
2. 8 邻域；
3. 正确处理直线/对角代价；
4. 明确 corner-cutting 策略；
5. start 来自当前 TF/localization；
6. goal 来自 geometry_msgs/PoseStamped；
7. 输出 nav_msgs/Path；
8. Path frame 与 planning map 一致；
9. 支持 SUCCESS / INVALID_START / INVALID_GOAL /
   NO_PATH / MAP_UNAVAILABLE / TF_UNAVAILABLE；
10. A* 核心与 ROS Node 尽量分离；
11. 添加最小单元测试。

禁止：
Hybrid A*
Nav2 Planner Plugin 重构
动态规划
机械臂
深度相机逻辑。

先列修改文件，再实施。
最后给出 build/test/RViz 验证。
```

## 推荐模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 8. Phase 4：Path Post-processing

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

第一版禁止 spline/复杂优化器。

所有 shortcut 必须在 inflated grid 上碰撞检查。

------------------------------------------------------------------------

## Codex Prompt

``` text
首先读取 AGENTS.md。

为 A* 增加轻量 Path Post-processing。

只实现：
1. 删除重复/共线点；
2. 可配置 line-of-sight shortcut；
3. shortcut 在 inflated grid 上碰撞检查；
4. 固定间距重采样；
5. 根据相邻点计算 yaw；
6. 输出 nav_msgs/Path；
7. raw path 和 processed path 都可以调试/RViz查看。

不要加入：
spline
优化器
复杂速度规划
第三方规划库。
```

## 推荐模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 9. Phase 5：LQR 数学模型审查

## 目标

**先设计，不直接大量写代码。**

必须明确：

-   differential-drive kinematic model；
-   error definition；
-   `e_y`；
-   `e_yaw`；
-   是否需要 `e_x`；
-   control input；
-   `v_ref`；
-   dt；
-   A/B；
-   Q/R；
-   nearest point；
-   lookahead；
-   terminal behavior。

------------------------------------------------------------------------

## Codex Prompt

``` text
首先读取 AGENTS.md。

本阶段暂时不要直接写大量 LQR 代码。

根据当前真实差速底盘、odom/TF接口和 nav_msgs/Path，
设计第一版低速 LQR path tracker。

请输出：
1. 差速车运动学模型；
2. 路径跟踪误差定义；
3. 状态向量；
4. 控制输入；
5. 连续/离散模型；
6. A/B 矩阵；
7. Q/R 含义；
8. v_ref 处理；
9. nearest/lookahead；
10. 终点减速与停车；
11. 需要读取的现有参数；
12. 最小代码结构。

目标：
低速、可解释、代码量小、方便Gazebo和后续实车调试。

不要复制参考公司项目的完整工业控制逻辑。
不要加入：
MPC
复杂 jerk 规划
换挡
brake hold。
```

## 推荐模型

`GPT-5.6 Sol / High`

## 验收

人工确认数学模型后才能进入 Phase 6。

------------------------------------------------------------------------

# 10. Phase 6：LQR 实现

输入：

``` text
nav_msgs/Path
TF/Odometry
```

输出：

``` text
geometry_msgs/Twist
raw command topic
```

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

至少记录：

``` text
x/y/yaw
target_index
e_y
e_yaw
v_cmd
w_cmd
goal_distance
controller_state
```

------------------------------------------------------------------------

## Codex Prompt

``` text
首先读取 AGENTS.md。

按照已经确认的 Phase 5 数学模型实现 LQR path tracker。

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
10. path丢失/TF异常时输出0；
11. 必要调试日志。

不要添加：
MPC
复杂纵向速度规划
jerk状态机
换挡
brake hold
深度相机。

给出build、运行和调参方法。
```

## 推荐模型

`GPT-5.6 Sol / Medium-High`

------------------------------------------------------------------------

# 11. Phase 7：2D LiDAR Safety Gate

A\* + LQR 不能替代实车安全停车。

数据流：

``` text
LQR raw cmd
    ↓
Safety Gate ← LaserScan
    ↓
base cmd_vel
```

第一版只做：

-   front sector；
-   minimum range；
-   stop distance；
-   release distance；
-   hysteresis；
-   LaserScan timeout；
-   invalid scan → stop。

不做局部绕障。

------------------------------------------------------------------------

## Codex Prompt

``` text
首先读取 AGENTS.md。

实现独立的 2D LiDAR Safety Command Gate。

输入：
LQR raw Twist
现有 LaserScan topic

输出：
现有底盘速度命令 topic

要求：
1. 前向扇区参数化；
2. stop_distance参数化；
3. release_distance参数化；
4. 简单hysteresis；
5. LaserScan timeout/invalid data时自动模式输出0；
6. Safety逻辑与LQR解耦；
7. 提供enable参数方便Gazebo调试。

不要实现：
DWA
TEB
局部绕障
深度相机安全层。
```

## 推荐模型

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 12. Phase 8：单目标 Gazebo 闭环

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

至少测试：

1.  直线；
2.  90° 转弯；
3.  多拐点；
4.  静态障碍绕行；
5.  窄通道；
6.  invalid goal；
7.  NO_PATH；
8.  LaserScan safety stop。

记录：

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

推荐：

``` text
常规集成：GPT-5.6 Terra / Medium
复杂跨模块 Bug：GPT-5.6 Sol / Medium-High
```

------------------------------------------------------------------------

# 13. Phase 9：Pickup → Delivery Task

单目标导航稳定后再加入 Task。

状态机：

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

-   到 pickup 即认为取货完成；
-   到 delivery 即认为放货完成；
-   不调用机械臂。

Task 只负责：

``` text
state
goal
navigation result
```

禁止复制 A\*/LQR 逻辑。

推荐：

`GPT-5.6 Terra / Low-Medium`

------------------------------------------------------------------------

# 14. Phase 10：Planning Failure 接口

## 目标

把 `NO_PATH` 变成后续 Environment Interaction 的正式入口。

NO_PATH 至少保存：

``` text
timestamp
start
goal
raw grid
inflated grid
planning result
```

后续再增加：

``` text
blocking region
clearance
candidate movable obstacle
```

第一版不要过度设计。

架构：

``` text
Planner
├── SUCCESS -> Controller
└── NO_PATH -> Future Failure Analyzer
```

推荐：

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 15. Phase 11：在现有 Camera Link 上添加 Gazebo Depth Plugin

**到这一阶段才添加。**

Phase 0/AGENTS.md V2 已经记录现有 camera link 的真实信息。

流程：

``` text
existing camera link
 ↓
Gazebo depth sensor/plugin
 ↓
RGB
Depth
CameraInfo
(optional PointCloud2)
```

如果现有 camera link 正确：

**禁止重新创建或移动它。**

如 ROS 相机坐标约定需要 optical frame，可以增加必要 optical
frame，但必须说明坐标变换原因。

第一阶段只验收：

``` text
RGB visible
Depth visible
CameraInfo valid
TF correct
RViz correct
```

不做：

-   YOLO；
-   segmentation；
-   movable classification；
-   机械臂；
-   3D SLAM；
-   MID360。

------------------------------------------------------------------------

## Codex Prompt

``` text
首先读取 AGENTS.md。

现在开始为 Gazebo 加入深度相机仿真。

重要：
URDF 已经存在预留 camera link。
使用 AGENTS.md 中 Phase 0 已确认的真实 link/frame 信息。

先确认：
1. camera link；
2. parent；
3. joint；
4. xyz/rpy；
5. TF；
6. optical frame。

然后以最小修改方式在现有 camera link 上添加 Gazebo depth sensor/plugin。

目标：
- RGB
- Depth
- CameraInfo
- 正确TF
- RViz验证

PointCloud2：
如果当前插件可以低成本直接提供则保留；
否则不要为了PointCloud2扩大修改范围。

不要实现：
YOLO
机械臂控制
3D SLAM
MID360。
```

推荐：

`GPT-5.6 Terra / Medium`

------------------------------------------------------------------------

# 16. Phase 12：Depth 障碍补盲

目标是补充单线 LiDAR 容易漏掉的：

-   低矮障碍；
-   高于扫描平面的障碍；
-   悬空结构。

第一版：

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

再与 2D LiDAR/地图障碍信息融合。

重要原则：

> 深度相机失效时，2D LiDAR 导航 baseline 仍能独立运行。

推荐：

`GPT-5.6 Sol / Medium`

------------------------------------------------------------------------

# 17. Phase 13：Environment Interaction 最小 Demo

目标场景：

``` text
Robot
 ↓
Blocked Corridor
 ↓
A*
 ↓
NO_PATH
 ↓
Depth Camera
 ↓
Detect Known Box
 ↓
Estimate Position
 ↓
Movable Candidate
 ↓
Predefined Manipulation Primitive
 ↓
Obstacle Moved
 ↓
Update Environment
 ↓
A* Replan
 ↓
SUCCESS
 ↓
LQR Continue
```

第一版优先：

-   已知箱子；
-   少量目标；
-   简单推/移；
-   预定义 manipulation primitive；
-   不做通用抓取；
-   不做 RL；
-   不做 VLA；
-   不做完整 affordance learning。

先验证：

> 传统导航失败后，机器人能够主动改变环境，并恢复路径可通行性。

推荐：

``` text
架构/算法：GPT-5.6 Sol / High
普通 ROS 接线：GPT-5.6 Terra / Medium
```

------------------------------------------------------------------------

# 18. 当前明确不做

导航 baseline 阶段不做：

-   MID360；
-   3D LiDAR；
-   完整 Nav2 重构；
-   Hybrid A\*；
-   DWA；
-   TEB；
-   MPC；
-   复杂动态障碍预测；
-   深度相机 Gazebo plugin；
-   PCL 大型点云处理；
-   YOLO；
-   机械臂抓取；
-   RL；
-   VLA；
-   行为树大框架；
-   工业 AMR 全套 recovery。

目标始终是：

> 先建立一个足够简单、可理解、可测试、能作为后续实验 baseline
> 的导航系统。

------------------------------------------------------------------------

# 19. Codex 全局执行规则

这些规则应该同时存在于 `AGENTS.md` 和每个关键 Phase Prompt
的简化版本中：

``` text
1. 开始任务前先读取根目录 AGENTS.md；
2. 再读取本 Phase 相关文件；
3. 不修改本 Phase 无关文件；
4. 不自行扩大需求；
5. 优先 ROS 2 标准消息；
6. 不创建自定义 Path.msg；
7. 参数优先 YAML；
8. 核心算法与 ROS wrapper 尽量分离；
9. 不猜 topic/frame/机器人参数；
10. 参考 xiache_projector 时只借鉴成熟设计思路，
    不复制其复杂业务逻辑；
11. 输出修改文件列表；
12. 输出修改前后数据流；
13. 输出 build 命令；
14. 输出 test/运行命令；
15. 输出验收结果；
16. 前置条件不满足时停止扩大修改并报告；
17. 每个 Phase 建议独立 git commit。
```

------------------------------------------------------------------------

# 20. 模型使用策略

目标：控制 Codex 额度，把高能力模型留给真正困难的问题。

  任务                             推荐
  -------------------------------- ------------------
  AGENTS.md V1/V2                  Luna Low
  小型 CMake/package/launch 修改   Luna/Terra Low
  仓库审计                         Terra Medium
  ROS 接口设计                     Terra Low-Medium
  Grid / Inflation                 Terra Medium
  A\*                              Terra Medium
  Path Processing                  Terra Medium
  LQR 数学设计                     Sol High
  LQR 实现                         Sol Medium-High
  普通集成                         Terra Medium
  TF/控制跨模块疑难 Bug            Sol Medium-High
  Depth Plugin                     Terra Medium
  Depth 障碍融合                   Sol Medium
  Environment Interaction 架构     Sol High

原则：

``` text
机械性修改
→ Luna

一般 ROS/C++ 工程与常规算法
→ Terra

数学模型、复杂控制、跨模块疑难问题
→ Sol
```

不要默认所有任务都使用 Sol High。

------------------------------------------------------------------------

# 21. AGENTS.md 的后续维护规则

`AGENTS.md` 不是一次生成后永远不动。

只在以下情况更新：

### 应更新

-   topic/frame 正式确定；
-   package 职责正式变化；
-   新增长期模块；
-   build/test 流程正式改变；
-   深度相机正式加入系统；
-   机械臂正式进入主系统；
-   发现一条 Codex 经常违反、且需要长期固定的工程规则。

### 不应更新

-   临时 bug；
-   某次测试失败；
-   某次实验参数；
-   某个 waypoint；
-   临时 Q/R；
-   一次性的 TODO；
-   某次终端日志。

这些应该进入 issue、实验记录或 Phase 报告。

------------------------------------------------------------------------

# 22. 第一阶段完成定义

导航 Baseline 完成时必须能够：

``` text
Gazebo
 ↓
Map / Localization
 ↓
Goal
 ↓
Inflated Grid
 ↓
A*
 ↓
Processed Path
 ↓
LQR
 ↓
2D LiDAR Safety Gate
 ↓
Differential Drive
 ↓
Goal Reached
```

并且能够稳定复现：

``` text
Blocked Path
 ↓
A*
 ↓
NO_PATH
```

`NO_PATH` 是下一阶段环境交互的入口。

------------------------------------------------------------------------

# 23. 当前立即执行顺序

现在不要直接写 A\*。

按下面顺序：

``` text
① Step -1
让 Codex 创建根目录 AGENTS.md V1

② 人工检查 AGENTS.md
确认没有把未知接口写死

③ Phase 0
让 Codex 只读审计 urdf_change

④ Step 0.5
让 Codex 把 Confirmed Repository Facts
写回 AGENTS.md V2

⑤ 人工检查 AGENTS.md V2

⑥ Phase 1
开始第一批实际代码修改

⑦ Phase 2 → Phase 10
完成纯 2D LiDAR 导航 baseline

⑧ Phase 11+
再进入深度相机与环境交互
```

当前最重要的是：

> **AGENTS.md 先约束 Codex 的行为；Phase 0 再建立真实仓库事实；之后才让
> Codex 修改算法代码。**

这样能够最大程度减少错误假设、无关重构和重复 Token 消耗。
