# 避障、移障与复原（模拟后端）

本功能在原取货、送货、返回 home 流程内增加路段检查和障碍恢复。所有新增源码、配置、测试及运行产物均在 `src/task`。原 `task.launch.py` 保留，原任务点位文件不会自动改写。

## 已增加的功能

- 新启动入口统一启动 Gazebo、AMCL、规划、LQR、Safety Gate 和恢复任务节点；规划器输出 `/task/candidate_path`，task 检查并向 `/plan` 放行执行片段，底盘仍只接收 Safety Gate 的 `/cmd_vel`。
- 对折线路段按扫描时刻的 TF 检查通行走廊；长段分批观察，未知/遮挡不当作空闲，行驶期间继续检查。原任务到达与中间片段到达分别关联。
- 维护 `/task/working_map` 临时障碍层，原 `/map` 继续提供给 AMCL。复用现有 `plan` C++ 核心的 Python 绑定评估绕行、持物通过后复原、临时放置后复原；记录时间/做功估计及候选淘汰原因。
- 用登记的 Gazebo 模型模拟类别、位姿和虚拟末端。操作通过 Gazebo 状态服务真实改变模型位置；观察目标随抬升移动，有限重试，保存首次原位姿，复原后验证误差。持物时不抓第二件物体；后方障碍需要重新观察。
- 保留原 `/task/start`、状态、事件、JSONL 日志。复原无法完成时记录未解决状态并拒绝开始新一轮。

类别策略、仿真操作范围、代价和超时在 `config/recovery.yaml`；登记模型与故障注入在 `config/recovery_scenarios.yaml`。默认袋子可移动且可以持续夹持，凳子不可移动，箱子需要临时放置。质量是配置输入，做功是估算，不是力传感器测量。

## 构建与测试

所有命令在 `car_ws` 根目录执行。重建 `plan` 是为了静态库具备 Python 扩展所需的 PIC；没有修改其源码。不要与原导航启动入口同时运行。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$PWD/src/task/logs/ros"
export GAZEBO_LOG_PATH="$PWD/src/task/logs/gazebo"
export TMPDIR="$PWD/src/task/logs/tmp"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH" "$TMPDIR"

colcon --log-base src/task/.colcon/log build \
  --build-base src/task/.colcon/build \
  --install-base src/task/.colcon/install \
  --packages-select plan controller car_description task \
  --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
source src/task/.colcon/install/setup.bash

colcon --log-base src/task/.colcon/log test \
  --build-base src/task/.colcon/build \
  --install-base src/task/.colcon/install \
  --packages-select task
colcon test-result --test-result-base src/task/.colcon/build --verbose
```

在新终端也加载相同 overlay 并设置上述日志环境变量。若执行相关回归，把测试的 `--packages-select task` 改为 `--packages-select plan controller task`；既有控制器/几何校验失败不能通过放松保护或修改范围外文件消除。

## 启动与测试障碍

1. 在 `config/task_points.yaml` 填写当前地图中实际可行的 home、pickup、dropoff，完成后设置 `configured: true`。每轮 start 重新读取此文件。
2. 启动完整恢复系统，显式传入源码配置便于调整后重启：

```bash
ros2 launch task obstacle_recovery.launch.py \
  points_file:="$PWD/src/task/config/task_points.yaml" \
  recovery_file:="$PWD/src/task/config/recovery.yaml" \
  scenarios_file:="$PWD/src/task/config/recovery_scenarios.yaml"
```

3. 在 RViz 设置有效 `/initialpose`，确认地图与场景对齐、TF、里程计及 `/scan` 就绪。地图坐标不等于 Gazebo world；工具使用同时间参考实体和 TF 转换，不创建 identity 静态变换。
4. 在原路径的空闲位置放置登记物体。下面 `2.0, 1.0` 仅是命令格式示例，必须换成当前场景中观察到的路径坐标：

```bash
ros2 run task recovery_scenario spawn bag \
  --registry "$PWD/src/task/config/recovery_scenarios.yaml" \
  --x 2.0 --y 1.0 --yaw 0.0 --frame map

ros2 service call /task/start std_srvs/srv/Trigger "{}"
```

场景名可选 `bag`、`stool`、`temporary`、`occluded`、`grasp_failure`，分别测试持物复原、不可移动物绕行、临时放置、后方遮挡和抓取失败。`--frame world` 明确使用 Gazebo 原生坐标。工具仅操作登记的测试模型，重复生成已有模型会报错。新场景建议重新启动仿真，避免上一场景物体残留影响比较。

`start_sim:=false` 复用已运行的兼容 Gazebo 和 map server，但仍由本入口启动 AMCL 和规划控制节点；已有导航节点应先退出。Gazebo 必须已经加载 `/task/gazebo` 命名空间的 `libgazebo_ros_state.so`。新入口会在 `src/task/logs/runtime/recovery.world` 生成 world 副本，原 world 保持只读；副本解析原文件相对资源位置，并仅增加状态插件。

## 配置与观测

新增的主要接口：

| 接口 | 类型 | 用途 |
|---|---|---|
| `/task/candidate_path` | `nav_msgs/Path` | A* 完整候选路径，仅 task 接收 |
| `/plan` | `nav_msgs/Path` | task 放行的路径片段，LQR 接收 |
| `/task/working_map` | `nav_msgs/OccupancyGrid` | 静态地图叠加临时障碍，供原膨胀节点使用 |
| `/task/gazebo/get_entity_state` | `gazebo_msgs/srv/GetEntityState` | 模拟识别及操作后的实际状态查询 |
| `/task/gazebo/set_entity_state` | `gazebo_msgs/srv/SetEntityState` | 虚拟抓取、搬运、放回 |
| `/task/recovery/markers` | `visualization_msgs/MarkerArray` | 在 RViz 添加 MarkerArray 显示 |

原 `/task/status`、`/task/events`、`src/task/logs/tasks.jsonl` 仍用于任务观测。还应一起检查 `/cmd_vel_raw`、`/cmd_vel`、`/controller/lqr/status`、`/controller/lqr/diagnostics`、`/controller/safety/status` 和 `/controller/safety/min_range`。`RAW_ZERO` 表示上游零速，`CLEAR` 不代表底盘正在运动。

配置加载顺序为节点自己的 YAML → 原 `navigation_baseline.yaml` → task 的路由覆盖。规划膨胀半径和安全余量使用 `recovery.yaml` 的 `planning` 配置，与候选评估保持一致；默认保留部署值 0.355 m 和 0.10 m。Safety Gate 继续加载自己的 YAML，不通过恢复配置降低其阈值。运行时用 `ros2 param get` 验证实际值，参数调整后重启；不要把 `recovery.yaml` 当成 ROS 节点参数 YAML 直接传给 `--params-file`。

## 边界与实际验证

本阶段没有 YOLO 推理、真实机械臂控制、接触力学或物理夹取验证。虚拟末端移动 Gazebo 模型，成功表示新鲜模型状态满足模拟抬升/复原条件。模拟 arm_reach、持物高度和包络不代表真实机器人能力；规划膨胀距离不等于雷达停止距离或经验证的物理安全余量。未知目标不能抓取；识别也受扫描可见性限制。

默认使用现有 Gazebo 和 Safety Gate 同样的 `+Inf` 无回波约定：仅证明 `range_max` 内无返回，不代表超量程空间空闲；NaN、负无穷、越界值及遮挡仍为未知。其他传感器应先确认其无回波语义，再配置 `allow_positive_infinity`。

未完成复原保存在 `src/task/logs/pending_restoration.json`，重启或再次 start 不会清除。需要先将登记物体恢复到日志中的原始 map 位姿，并让节点获得连续的新鲜可见观测；不要通过删除账本掩盖未复原物体。

截至 2026-10-10，四包构建与最终 task 重建通过；task 全量 64 个 pytest 用例和 8 个 CTest 测试组全部通过，耗时 4 分 11 秒。新增部分包括 35 个基础用例和 6 个联动用例：无障碍、袋子复原、临放箱子复原、凳子绕行、第三次抓取成功、扫描丢失停稳。联动使用真实 A*、LQR、Safety Gate，加理想底盘与 Gazebo 兼容状态服务。`colcon test-result` 含测试组汇总为 72 项，0 errors、0 failures、0 skipped。详细进度见工作区 `docs/task.md`，测试日志见 `src/task/logs/final_task_tests.log` 和 `task_results.log`。

相关 plan/controller 回归存在两个范围外的既有失败：`verify_robot_radius` 不支持当前模型的 `arm_1_joint`；Safety Gate 测试在 0.4 m 期待停止，但已有 YAML 停止距离为 0.2 m。未修改这些文件或调整安全阈值。

**真实 Gazebo 验收尚未完成。** 已尝试新入口，沙箱拒绝本地通信套接字，gzserver 也无法创建其固定用户日志目录 `/home/dx/.gazebo/server-11385`。本地通信测试之后获准执行，但没有授权写入 task 范围外的 Gazebo 用户目录，因此未再次启动 Gazebo。`GAZEBO_LOG_PATH` 不能重定向此次报错的控制台目录。现场运行前需解决该目录写入与当前修改范围的冲突；定位对齐、实际激光变化和四种物理场景闭环仍待验收，不能用理想模型测试代替。失败启动日志在 `src/task/logs/gazebo_acceptance_launch.log`。
