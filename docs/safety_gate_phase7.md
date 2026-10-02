# Phase 7 — 2D LiDAR Safety Command Gate

## 接口与范围

独立 executable `controller/safety_gate_node`，节点名 `safety_gate`。
不修改 LQR、规划器、URDF/Gazebo，也不引入任何新依赖。

| 数据 | 默认接口 | 类型 / QoS |
| --- | --- | --- |
| 输入 raw command | `/cmd_vel_raw` | `geometry_msgs/msg/Twist`，reliable / volatile / depth 1 |
| 输入 Scan | `/scan` | `sensor_msgs/msg/LaserScan`，best_effort / volatile / depth 5 |
| 方向转换 | TF `base_footprint <- laser_link` | 标准 TF，在 Scan 时间戳查询 |
| 输出底盘命令 | `/cmd_vel` | `geometry_msgs/msg/Twist`，reliable / volatile / depth 1 |
| 状态 | `/controller/safety/status` | `std_msgs/msg/String`，reliable / transient_local / depth 1 |
| 最小前向距离 | `/controller/safety/min_range` | `std_msgs/msg/Float64`，m，reliable / volatile / depth 1 |

```text
LQR -- /cmd_vel_raw --> Safety Gate <-- /scan + base/laser TF
                             |
                          /cmd_vel
                             v
                  existing DiffDriveController
```

Safety 核心 `safety_gate.hpp/.cpp` 不依赖 ROS 或 LQR。
wrapper 负责输入有效性、时间、TF、Twist 和发布；此层仅放行或停止，不做绕障。

## 方向、距离与 hysteresis

仓库 `laser_joint` 的固定 yaw 是 **3.14 rad**，位置是
`base_link` 中 `(0.155, 0, 0.1855) m`；`base_link` 又比 `base_footprint` 高 0.097 m。
不能以 LaserScan 的零角度直接当作机器人前向。
节点从真实 TF 提取平面 yaw，将每条射线方向变换到 `base_frame`。
扇区以底盘 +x 为中心，默认半角 `0.785398... rad`（±45°）。
不在源码中猜测或硬编码上述外参；缺失 TF、非平面姿态或非单位四元数都停止。

距离是 **LiDAR 原点沿射线的 range**，不是机器人中心距离，也不是车体外沿净空。
扇区使用变换后的射线方向，未把障碍物坐标改成底盘原点的方位角。
默认 `stop_distance=0.50 m`、`release_distance=0.65 m`。

- 启动为 blocked；有效扫描最小距离达到 `release_distance` 才解锁。
- `min_range <= stop_distance`：blocked，输出完整零 Twist（v 和 w 都为 0）。
- `min_range >= release_distance`：解锁，允许有效、新鲜 raw command。
- 两阈值之间：保持先前状态，避免阈值附近来回抖动。
- Scan 无效、过期、时钟失效时重新锁定；恢复必须再次达到释放距离。

必须满足 `release_distance > stop_distance > 0`。
这些是低速仿真初始参数，不是已经实测确认的安全停车距离。
调试时需考虑雷达相对车体位置、机器人外轮廓、检测/发布/控制延迟、实际制动距离
与额外裕量。此处没有引入速度相关距离规划。

## 有效数据与失效停车

- Scan frame 必须等于配置的 `scan_frame`；时间戳必须非零、不超时，
  超前不能大于 `future_tolerance_sec`。
- 检查有限、正的角分辨率、角度终点与数组长度一致、测距范围合法。
  扇区边界和相邻射线间隙必须满足 `max_beam_gap`，不把扫描盲区当作自由空间。
- 扇区内任一 NaN、负无穷、低于 range_min 或高于 range_max 的有限值：无效、停车。
  后方扇区外的坏测距不影响本层的前向判断。
- `allow_positive_infinity=true` 显式采用仿真 no-return 约定：
  +Inf 表示直到 `range_max` 没有回波；全 +Inf 扫描也可有效。
  这不包括 NaN 或负无穷。必须在实际传感器上确认此语义；
  不确认或希望更严格时设为 false，此时前向 +Inf 也停车。
- 不把空数组、只有坏值或扇区缺失的 Scan 当作畅通。
- Scan 同时检查 ROS 时间戳和 steady-clock 接收时间，默认 `scan_timeout_sec=0.30`。
- Twist 没有消息时间戳，接收时记录 ROS/steady 时间；默认 `raw_timeout_sec=0.20`。
  NaN/Inf、非平面分量、倒车或超出 v/w 上限都拒绝，而不是裁剪后放行。
  本 baseline 的 LQR 只前进；本层不支持倒车安全，不能用于遥控反向通行。
- 输出 wall timer 默认 50 Hz；ROS `/clock` 未开始、暂停超过 0.30 s，
  或时间回跳/大跳变都会零输出。时钟跳变清除旧输入，需要重新接收。
- 有效零 raw command、障碍/无效 Scan 在回调内立即零输出；
  超时在下一个 wall timer tick 检查（有调度延迟，不是硬实时急停）。
- status 变化时打印日志；典型状态为 `CLEAR`、`OBSTACLE_STOP`、
  `WAIT_SCAN`、`WAIT_RAW`、`SCAN_INVALID`、`SCAN_TIMEOUT`、`TF_UNAVAILABLE`、
  `RAW_INVALID`、`RAW_TIMEOUT`、`CLOCK_RESET`、`CLOCK_UNAVAILABLE`、`DISABLED`。
  `RAW_ZERO` 表示收到零命令，下一次正常 tick 状态可回到 `CLEAR`，输出仍为零。
  `min_range` 只随 Scan 更新，无效 Scan 发布 NaN；不能把最后的数值当作新鲜 Scan。

`enable=false` 的定义是 **禁止自动运动，持续输出零**，不是不安全的 raw passthrough。
参数启动时只读；改 YAML/launch 参数后重启。禁用节点仍发布零，
不能因此和 teleop 同时控制底盘。需要遥控时停止自动命令发布节点并明确切换模式。

## 构建与测试

所有操作在仓库根目录；生成文件也保持在仓库内：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/phase7_safety_gate build --base-paths src \
  --build-base build/phase7_safety_gate --install-base install/phase7_safety_gate \
  --packages-select controller plan
source install/phase7_safety_gate/local_setup.bash
mkdir -p log/phase7_safety_gate/ros
export ROS_LOG_DIR="$PWD/log/phase7_safety_gate/ros"
colcon --log-base log/phase7_safety_gate test \
  --build-base build/phase7_safety_gate --install-base install/phase7_safety_gate \
  --packages-select controller plan --event-handlers console_direct+
colcon test-result --test-result-base build/phase7_safety_gate --verbose
git diff --check
```

测试覆盖仓库实际雷达 yaw、前后扇区、阈值与 hysteresis、Scan 空/坏/缺失，
超时、TF 缺失、raw 错误/失联、+Inf 策略、禁用状态、暂停/跳变时钟，
并回归已有 LQR / planning grid / A* / post-processing。
ROS 测试使用独立 DDS domain，**且底盘输出 remap 到 `/gate_test/out`**；
不会向当前仿真的 `/cmd_vel` 发送运动命令。

## Gazebo 运行与调试

先验证实际 Gazebo 接口与 TF，再部署。地图/定位的缺失不由 Safety Gate 补齐。
自动导航前仍须满足 Phase 6 的真实规划 frame、Path、TF、Odometry 前置条件。
以下启动 Gate 的命令只在已准备自动模式、移除其他底盘命令发布方后执行：

```bash
# 各终端 source 上述 ROS/overlay，设置仓库内 ROS_LOG_DIR
ros2 launch controller safety_gate.launch.py use_sim_time:=true enable:=true
# LQR 仍由已有 lqr_tracker.launch.py 启动；planning_frame 用实际已确认值

# 只读接口验证
ros2 topic info /cmd_vel --verbose
ros2 topic info /cmd_vel_raw --verbose
ros2 topic info /scan --verbose
ros2 run tf2_ros tf2_echo base_footprint laser_link
ros2 topic echo /controller/safety/status
ros2 topic echo /controller/safety/min_range
ros2 topic echo /cmd_vel
```

不要再直接向 `/cmd_vel` pub 非零命令；teleop 和其他控制节点必须退出。
自动模式 Safety Gate 应为底盘命令的唯一发布方。
门控无法防止其他 publisher 绕过它，也不能覆盖进程崩溃后的行为；
现有 diff_drive_controller 的 `cmd_vel_timeout=0.5 s` 是进程失联的后备停止机制，
不是该 Gate 的检测周期。

先隔离输出观察，不连接底盘，可使用：

```bash
ros2 run controller safety_gate_node --ros-args \
  --params-file src/controller/config/safety_gate.yaml \
  -r /cmd_vel:=/safety_debug/cmd_vel
ros2 topic echo /safety_debug/cmd_vel
```

RViz：Fixed Frame 选 `base_footprint`，添加 TF 和 `/scan` LaserScan，
检查车头对应的真实射线方向；同时观察 `min_range`、status 和隔离输出。
Gazebo 低速验证时，将障碍放在实际前向扇区内，确认停止，再移到释放距离外。
停止 `/scan` publisher 验证 Scan 超时、停止 LQR 验证 raw 超时；
暂停 Gazebo 验证持续零输出，恢复后输入就绪才恢复。
调 YAML 中扇区角、stop/release、超时和 v/w 上限，重启生效；
Gate 上限必须与 LQR 保持一致（可更严格，但超限 raw 将被拒绝）。

这个简单前向层不保证侧后方、原地旋转扫掠、低于/高于雷达平面的障碍检测，
不检查完整车体 swept footprint，也不是经认证的安全系统。
没有 DWA、TEB、绕障、3D/depth/PointCloud2 或完整 Nav2 costmap。
