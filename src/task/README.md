# 基础取送货任务

本阶段实现：每轮锁定 YAML 点位 → 取货点到点对齐停稳 → 模拟取货 3 秒 → 按到取货点的直线距离一次排序送货点 → 每点对齐停稳、模拟放货 3 秒 → 返回 home 对齐 → 等待下一次 start。无真实机械臂动作，无自动重试、跳站或断点恢复。

新增内容主要在 task；plan 最小增加末点目标朝向和关联诊断；controller 增加最终转向、到达反馈、禁止运动接口及诊断。Safety Gate、URDF、原导航 launch 未修改。

## 构建与测试

以下命令均在 `car_ws` 根目录执行。产物和 ROS 日志放在 task 内；不写原工作区 build/install/log。不要混用旧终端里已经运行的导航节点，先结束旧运行，再从新 overlay 启动。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/src/task/logs/ros"
export PYTHONDONTWRITEBYTECODE=1
export TMPDIR="$PWD/src/task/logs/tmp"
mkdir -p "$TMPDIR"
colcon --log-base src/task/.colcon/log build \
  --build-base src/task/.colcon/build --install-base src/task/.colcon/install \
  --packages-select plan controller task
source src/task/.colcon/install/setup.bash
colcon --log-base src/task/.colcon/log test \
  --build-base src/task/.colcon/build --install-base src/task/.colcon/install \
  --packages-select plan controller task
colcon test-result --test-result-base src/task/.colcon/build --verbose
```

## 配置与启动

1. 编辑 `src/task/config/task_points.yaml`，用当前地图中确认可行的点替换示例。一个 home、一个 pickup、至少一个 dropoff；所有 name 必须唯一，x/y 单位米，yaw 单位弧度，frame 必须是部署的 map。完成后设置 `configured: true`。示例默认不能启动任务，地图坐标不等同 Gazebo world 坐标。
2. 每个终端加载上述 ROS、原 install 和 task 内的新 overlay，并设置 ROS_LOG_DIR。
3. 启动现有导航：

```bash
ros2 launch car_description navigation.launch.py
```

4. 在 RViz 用 `/initialpose` 设置正确初始位姿，确认 map、TF、里程计、LiDAR 和 Safety Gate 就绪。
5. 独立启动任务节点，显式传入源码 YAML，后续编辑文件无需重新构建：

```bash
ros2 launch task task.launch.py \
  points_file:="$PWD/src/task/config/task_points.yaml" \
  params_file:="$PWD/src/task/config/task.yaml"
ros2 service call /task/start std_srvs/srv/Trigger "{}"
```

start 的 success 表示配置已接受，随后异步禁止运动、确认停稳、解除禁止并发送第一目标；不代表本轮已完成。每次 start 重新读取点位，整轮不可变。运行中重复 start 被拒绝。运行期间禁止其他节点/RViz发送 `/goal_pose`。

导航和 task 的 `planning_frame`、`base_frame`、`odom_frame`、到点/朝向/停稳阈值必须一致。task.yaml 参数只读，修改后重启 task；点位 YAML 则每轮重新读。默认导航和取放货使用仿真时钟；动作延迟基于 ROS 时间，规划、导航、停止与输入看门狗使用墙钟。时钟暂停不会完成动作，长暂停/回跳会终止本轮。

## 状态与接口

| 接口 | 类型 | 含义 |
|---|---|---|
| `/task/start` | std_srvs/Trigger | 读取并锁定一轮配置 |
| `/task/status` | std_msgs/String（JSON，持久 QoS） | 任务编号、状态、当前点、故障及停稳确认 |
| `/task/events` | std_msgs/String（JSON） | PICKUP_STARTED/DONE、DROPOFF_STARTED/DONE、DELIVERY_ORDER、TASK_DONE |
| `/goal_pose` | geometry_msgs/PoseStamped | task 唯一输出的导航目标，不提供起点 |
| `/controller/lqr/reached_goal` | geometry_msgs/PoseStamped | 已接受路径时间戳和末点位姿；控制器周期发布，task 去重 |
| `/controller/lqr/set_inhibit` | std_srvs/SetBool | true 清路径、持续零速并拒绝路径；false 只解除禁止，不恢复旧路径 |
| `/plan/diagnostics` | diagnostic_msgs/DiagnosticArray | 规划结果、detail、goal_stamp_ns、path_stamp_ns |
| `/controller/lqr/diagnostics` | diagnostic_msgs/DiagnosticArray | 状态、首故障 detail、path_stamp_ns、inhibited |

所有话题及服务名可在 YAML 配置或重映射。规划器对非零目标时间戳原样传递到 Path；零时间戳目标兼容旧用法，以规划时刻生成 Path 时间戳。task 每个目标使用非零时间戳，并只接受匹配路径及到达反馈。禁止接口记录时间屏障，解除后迟到的旧请求路径仍不会恢复运行。

路径终点仍是占据栅格中心，非原始任意浮点位置。0.05 m 方格的坐标量化偏差最大约 0.0354 m，另有到点容差；不等于实际定位/机械臂精度保证。现有途中拐角仍停车对齐。朝向转动也经过 Safety Gate，阻挡时可能无法转向。

## 故障与日志

默认 `src/task/logs/tasks.jsonl`，10 MiB/文件，5 个备份。记录任务 UUID、UTC 墙钟、ROS 时间、阶段耗时、目标位姿、配置快照、排序、完成事件和失败快照。日志不记录原始传感器流。SIGINT/SIGTERM 正常退出时会请求禁止运动并限时等待停稳；强制杀进程或断电无法执行此请求，任务节点本身不是硬件急停。磁盘写入失败会拒绝 start 或终止运行。

```bash
ros2 topic echo /task/status
ros2 topic echo /task/events
ros2 topic echo /plan/diagnostics
ros2 topic echo /controller/lqr/diagnostics
tail -f src/task/logs/tasks.jsonl
```

故障快照含 planner、LQR、Safety Gate、min_range、cmd_vel_raw、cmd_vel 及诊断和接收年龄；未收到字段为 unavailable，非有限距离写为字符串，不生成非法 JSON。LQR 首故障 detail 包含几何误差、阈值、位置、段号和真实速度。RAW_ZERO 只是上游零命令，不作为任务根因。

默认规划超时 10 秒，单站导航超时 300 秒；临时障碍解除后可继续，超过期限终止，不自动降低保护阈值。失败调用禁止接口，并等待其响应及新鲜停稳里程计。无法确认时显示 STOPPING、stop_confirmed=false 并输出 STOP_UNCONFIRMED，拒绝新 start；仍继续等待确认，恢复后进入 FAILED。检查实际机器人状态，排除故障后手动 start，新一轮从取货开始。

日志权限、磁盘空间、无路径、TF/里程计/时钟失效、路径清除、跟踪错误和机械臂模拟期间位姿丢失都会有具体原因。需要同时查看上下游诊断；CLEAR 不意味着机器人应当运动。

## 验证边界

单元和节点测试覆盖状态机、相关反馈、超时及停止等行为。测试通过不代表真实 Gazebo 场景已经闭环验收；地图与 world 对齐、初始位姿、真实误差及安全距离需针对运行场景验证。后续排序代价从 `task_runtime.core.order_dropoffs` 的 scorer 接口扩展，本阶段仅实现直线距离。

## 本次实际验证结果（2026-10-09）

- plan/controller/task 构建通过；安装后的 task Python 节点与源码一致，`ros2 launch task task.launch.py --show-args` 通过。
- task：12 项核心测试、10 项 ROS 节点测试、1 项真实 A*/LQR/Safety Gate 联动测试全部通过。联动测试使用理想差速底盘传感器模型，实际包含前进、转向和回 home，不是 Gazebo 验收。
- LQR：19 项核心测试、15 项节点测试通过；A* 节点 12 项测试通过。新代码的 Python/CMake 风格检查和三个包的 XML 校验通过。
- 完整 `colcon test` 仍有两项原有失败：Safety Gate 的旧测试在 0.4 m 期待停车，但 HEAD 中配置 stop_distance 为 0.2 m；`verify_robot_radius` 不支持 HEAD 中已有的 `arm_base_joint` prismatic 关节。相关源文件、配置、测试和 URDF 均未修改，不通过调阈值或改模型掩盖失败。半径校验失败意味着本次不能确认当前整机碰撞包络。
- `colcon test-result` 汇总为 206 tests、0 errors、3 failures、19 skipped；同一 Safety Gate 失败在 CTest 和 pytest XML 各计一次，因此实际失败检查是上述两项。cppcheck 因已安装 2.7 版本的工具规则跳过，不能视为已完成静态检查。
- ROS 联动测试在允许本机 DDS 通信的环境运行，ROS_DOMAIN_ID 与普通运行隔离。XML 校验采用 ROS 官方 `ros-infrastructure/rep` 仓库的 `package_format3.xsd`、`package_common.xsd` 本地副本；默认 HTTP schema 访问受限，HTTPS 下载站证书也不匹配。当前工作区可在执行上述 test 命令前设置：

```bash
export XML_CATALOG_FILES="$PWD/src/task/.colcon/xml_catalog.xml"
```

该 catalog 和 schema 只在忽略的构建目录中，未修改任何 package.xml 的 schema 地址。完整失败详情保留在 `src/task/.colcon/build/{controller,plan}/Testing/Temporary/LastTest.log`。
