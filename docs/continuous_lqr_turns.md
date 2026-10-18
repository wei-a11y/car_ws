# 连续圆弧跟踪与提前减速

本次沿用 `A* -> Path Processing -> LQR -> Safety Gate -> 底盘`，直线标称速度仍是
`0.15 m/s`。普通圆弧采样点连续通过；终点、圆滑失败的折角以及大误差恢复才会停车。
导航 launch 默认控制器改为 `lqr`；仍可显式选择 `controller_type:=dwa`，但 DWA 不使用本次连续 LQR 功能。

## Path 与辅助元信息

主接口保持 `/plan: nav_msgs/Path`。新增 `/plan/profile: common_msg/PathProfile`，包含
完整关联 Path、解析参考切线 `tangent_yaw`、有符号曲率 `curvature` 和 `stop_required`。
每个数组与 Path 点数一致；末点必须停车。圆弧的切线和曲率来自构造几何，
不通过可能含共线插值点的三点差分来估算。

标准 Path 没有曲率或停车点字段，单独按夹角猜测不能可靠区分小折角与圆弧采样点，
因此新增的是辅助语义消息，不创建或替换自定义 Path.msg。没有将标记编码到 z、stamp 或 quaternion 中。
辅助消息复制完整 Path 是为了比较完整 header、各 pose header、位置与姿态，
避免相同时间戳但不同路径被错误配对；只比较时间戳或数组长度不够。

两条 topic 保持 reliable/volatile/depth 1，订阅顺序任意。LQR 缓存最近一条 Path 和 profile，
仅在完整匹配、参数数组合法时执行；新消息不匹配时先输出零并等待配对。
数据丢失/跨 topic 快速替换可能需要重新发目标，不将等待误认为有效旧路径。
Path 或 profile publisher 消失都清空执行状态。新路径不继承旧命令或旧进度。
辅助 profile 不是心跳，不因路径创建较早自动过期；TF、Odometry、clock/watchdog 仍按原逻辑检查。

YAML 默认 `require_path_profile: true`。仅为兼容隔离旧接口测试保留 `false` 模式：
它沿用旧的逐折线停车策略，不具备本次圆弧连续通过能力。部署时应保持 true。

## 连续进度与停车边界

接收 profile 时计算累计弧长。每周期将真实 TF 位置投影到上次进度附近按米限制的窗口中，
进度单调推进，不积分发送的速度。允许跨过多个普通采样点，且不搜索/跨过下一必要停车点。
投影超限仍锁存 TRACKING_ERROR；路径自交不会通过全路径最近点搜索直接跳到远处。

仅在下一必要停车点实际 XY 距离达容差后输出零，并等待实测线/角速度停止。
中间停车点完成后对齐下一段；终点完成后锁存 GOAL_REACHED。
普通曲线不进入段尾 BRAKING；航向超过独立的 `tracking_align_trigger` 才触发大误差恢复。

参考切线在已验证弦段内按弧长插值，角差做 wrap；不能跨停车折角插值。
曲率按出发几何段读取，直线为零、圆弧为 `+/-1/R`。原 Path 的位置几何不在控制器内重新平滑。

## 速度与曲率协同

每个参考点取标称速度、角速度预算和侧向加速度约束的最小值：

`v_limit = min(v_nominal, angular_fraction*w_max/abs(kappa), sqrt(a_lateral/abs(kappa)))`

零曲率不构成曲率限速。入弯点同时检查前后曲率，必要停车点速度上限为零。
从整条路径末尾向前回推 `v_i <= sqrt(v_next^2 + 2*a_deceleration*ds)`，使紧弯和停车点
提前影响前方速度；普通采样点不被当作减速至零的目标。
运行时查询前视位置的制动包络，给一个控制周期的前进量留余量，再按实际线速度限制加速。
`slowdown_gain` 仅针对下一必要停车点距离，不再针对每条短弦的末端。

角速度由 `w = v*kappa - K_y*e_y - K_heading*e_heading` 生成。
参考曲率允许很短的前视，且不跨停车点；前馈只占 `w_max` 的一部分，剩余用于反馈。
跟踪角速度限制变化率；错误/急停、数据失效、停车对齐可以立即归零，不被舒适性限幅延迟。
制动包络与加速度参数是理想跟踪模型下的参考约束，不能保证定位突变、实际底盘滞后等情况下
每次速度命令的减速度都严格相同。

当前仍为相切圆弧，直线与圆弧处曲率有跳变，不声称曲率或角加速度处处连续。
短前视和角速度限幅会产生可控的参考跟踪误差，须结合膨胀余量验证；没有暗中生成新曲线。
固定 LQR 增益保留，低速变速/饱和曲线的表现通过非线性模型验证，不扩大为全局稳定保证。

## 调参位置

`src/controller/config/lqr_tracker.yaml`，所有值都是初始低速调参值，不是实测硬件能力：

| 参数 | 当前值 | 用途 |
| --- | --- | --- |
| `v_nominal` | 0.15 m/s | 保持原直线标称速度 |
| `linear_acceleration` | 0.10 m/s² | 出弯与起步加速度 |
| `linear_deceleration` | 0.15 m/s² | 制动包络，越小越早减速 |
| `angular_acceleration` | 1.0 rad/s² | 跟踪角速度变化限制 |
| `lateral_acceleration` | 0.04 m/s² | 弯道侧向加速度上限 |
| `feedforward_angular_fraction` | 0.75 | 前馈角速度占 w_max 的比例 |
| `feedforward_preview_distance` | 0.02 m | 曲率前馈/限速共同使用的短前视 |
| `tracking_align_trigger` | 1.0 rad | 大航向误差恢复阈值 |
| `max_reference_curvature` | 20 1/m | 拒绝异常曲率数据 |
| `reference_heading_tolerance` | 0.20 rad | 校验解析切线与 Path 弦方向一致性 |

`src/plan/config/astar.yaml` 的 `post_processing`：

| 参数 | 当前值 | 用途 |
| --- | --- | --- |
| `arc_radius` | 0.35 m | 期望半径 |
| `arc_min_radius` | 0.20 m | 可接受最小半径 |
| `arc_radius_max` | 0.65 m | 允许适当增加半径；0 表示上限为期望半径 |
| `arc_radius_weight` | 1.0 | 半径偏离代价 `(R-R_preferred)^2` 的权重 |
| `arc_curvature_weight` | 0.04 | `1/R` 代价权重，鼓励更舒展的弯道 |
| `recover_shortcut_corners` | true | shortcut 留下折角时比较原路线 |
| `retained_corner_cost` | 2.0 | 比较路线时每个保留拐点的等效米代价 |

候选始终逐一检查连续圆弧及输出弦的碰撞，半径变大不等于一定安全。
原先固定各占共享段一半，现在在初始分配后前后各再分配一次，用相邻圆弧实际占用量限制，
允许借用未圆滑邻角的空闲空间；不允许圆弧重叠。
存在回退时可比较 shortcut 前路线，代价为路径长度加保留拐点代价；
只选择验证过的候选，没有全局最优或一定消除全部折角的保证。
日志报告 rounded/retained、space limited/collision limited 和 shortcut recovered。

参数均为只读启动参数。修改 YAML 后重启相应节点即可，无需重新编译 C++；若使用普通安装
而非 symlink install，修改源码 YAML 后需重新安装该包，或通过 launch 的参数文件选项直接指定源码 YAML。

## 构建和验证

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/continuous_turn build --base-paths src \
  --build-base build/continuous_turn --install-base install/continuous_turn \
  --packages-select common_msg plan controller car_description --cmake-args -DBUILD_TESTING=ON
source install/continuous_turn/local_setup.bash
# 本环境沿用已下载的 ROS 官方 schema catalog，防止 xmllint 联网失败。
export XML_CATALOG_FILES="$PWD/log/arc_smoothing/schema_catalog.xml"
colcon --log-base log/continuous_turn test --build-base build/continuous_turn \
  --install-base install/continuous_turn --packages-select common_msg plan controller car_description
colcon test-result --test-result-base build/continuous_turn --verbose
```

catalog 的下载与创建说明见 [局部圆弧文档](path_arc_smoothing.md)。cppcheck 2.7 可能被 ament 自动跳过，
不能把跳过项当作已执行。仓库既有 DWA 文件若触发 lint，不为本任务格式化或改写其算法。

新增端到端核心测试由真实路径处理模块生成左右圆弧，在 0.20/0.35/0.60 m 半径下运行理想非线性
差速模型，检查无中间停车、保持直线速度、曲率限速/前馈、跟踪误差及最终到达。
ROS 测试验证两个 topic 顺序、错配、非法数组、publisher 消失和圆弧采样点连续通过。
已有 TF/Odometry/clock/watchdog 和 Safety Gate 测试继续保留。
这些是隔离合成测试，不表示已经通过 Gazebo 实际场景或硬件运行验收。

实际仿真可直接引用源码中的调参 YAML，重启节点即可读取，无需重新安装配置：

```bash
ros2 launch car_description navigation.launch.py controller_type:=lqr \
  planner_params_file:="$PWD/src/plan/config/astar.yaml" \
  lqr_params_file:="$PWD/src/controller/config/lqr_tracker.yaml"
```

前提仍是已有有效地图、定位、TF 和 scan。重新发送目标后检查日志的 profile 配对、必要停车索引、
曲率和前馈；同时观察线/角速度随时间的变化。XY 曲线重合不能证明过程无停车。

## 本次验证结果与已有阻塞

四个包构建成功。Path Processing 23 项、规划 ROS 节点 24 项、连续跟踪核心 5 项、
原 LQR 核心 17 项和 LQR ROS 节点 21 项全部通过。连续跟踪核心中的一个测试包含六种
左右转/半径组合。另一个双圆弧理想模型示例得到 0 次中途停车，最大投影误差 0.01 m；
图与原始日志在 `log/continuous_turn/continuous_turn.png`、`tracking.csv`。

完整 colcon 汇总为 233 条测试记录、0 errors、9 failures、24 skipped；CTest 与 xunit 会重复计数失败。
失败涉及未改动的 DWA 格式/文件末尾换行，以及原 Safety Gate 测试与当前配置不一致。
当前停车距离为 0.2 m，测试却以 0.4 m 期待停车；测试还把 0.3 m/s、0.7 rad/s 视为超速，
但当前 gate 配置允许 0.6 m/s、0.8 rad/s。没有为让旧测试通过而修改这些部署配置。
cppcheck 的跳过属于当前 ament 对 2.7 版本的处理。

还显式运行了 `src/urdf/test/test_navigation.py`：2 passed、3 failed。
其中 AMCL YAML 同时出现 `set_initial_pose: false` 和后置的 `true`，实际后者生效，
与测试“先无定位，等待显式初始位姿”的假设不一致；另一个障碍停车断言也受上述距离配置影响。
这些原有配置均保持不变，因此不能声称完整导航验收或 Gazebo 实际运行已经通过。

复现额外导航测试：

```bash
ROS_LOG_DIR="$PWD/log/continuous_turn/navigation_ros" /usr/bin/python3 -m pytest \
  src/urdf/test/test_navigation.py --basetemp log/continuous_turn/navigation_pytest \
  --junit-xml=log/continuous_turn/navigation_tests.xml -q
```
