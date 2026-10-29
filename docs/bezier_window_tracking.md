# 多拐角 Bézier、动态角加速度预算与速度协调

本功能为实验选项，默认继续执行原有路径处理和速度逻辑。`/plan` 保持
`nav_msgs/Path`，辅助 `PathProfile` 的消息结构不变。没有调整物理运动限值、
PID/LQR 权重、地图膨胀或 URDF/Gazebo 模型。

## 参数入口

所有操作参数均在 YAML 中，修改后重启 planner/controller。

| 文件 | 参数组 | 用途 |
| --- | --- | --- |
| `src/plan/config/astar.yaml` | `post_processing.enable_window_bezier` | 多拐角窗口优化开关，默认 false |
| 同上 | `post_processing.enable_turn_speed_coordination` | 抑制相邻低速区之间的多余速度峰，默认 false |
| 同上 | `window_merge_gap / window_extension / window_knot_spacing` | 窗口合并、边界扩展和内部节点间距 |
| 同上 | `window_max_deviation / max_length_ratio / window_max_time_ratio` | 偏移、长度和全局时间预算 |
| 同上 | `window_target_speed / window_target_radius / window_transition_scale` | 速度、有效半径与曲率过渡偏好；不强制最低指令速度 |
| 同上 | `window_*_weight / window_constraint_fraction` | 归一化目标和约束余量惩罚；最终仍独立硬校核 |
| 同上 | `window_derivative_* / window_heading_max_change / window_min_tangent_ratio` | 优化变量范围和正则性偏好 |
| 同上 | `window_solver_seconds / window_total_seconds / window_iterations` | 单窗口共享时间、全路径时间和单初值迭代预算 |
| 同上 | `window_seed_scales / window_objective_samples / window_max_knots / window_check_budget` | 确定性初值、目标积分密度、节点和校核资源上限 |
| 同上 | `coordination_*` | 速度峰合并距离、最小幅度、轮数及事件检测参数 |
| `src/plan/config/motion_limits.yaml` | `enable_dynamic_angular_budget` | planner/controller 共用动态预算开关，默认 false |
| 同上 | `dynamic_budget_utilization` | 名义规划预算利用率，默认 0.95，为测量速度滞后留余量；没有提高物理预算 |
| 同上 | `speed_envelope_rounds / speed_envelope_tolerance / speed_search_iterations` | 速度可达传播及在线约束搜索精度 |
| `src/controller/config/path_tracking.yaml` | `analysis.target_turn_speed_* / platform_speed_error_tolerance / acceptance_*` | 评估目标和验收阈值，不改变控制约束 |
| `src/plan/config/bezier_benchmark.yaml` | `benchmark.*` | 固定地图、路线、重复次数、仿真隔离域、初始位姿与超时 |

实验组合在 `astar.yaml` 中开启两个开关，并在共享 `motion_limits.yaml` 中开启动态预算。
三个开关相互独立，基线对照必须全部关闭。不要在其他 YAML 中重复物理运动参数。
旧 YAML 缺少新增参数时使用核心默认值，实验功能保持关闭。

## 算法和边界

先运行原有完整路径处理，冻结同地图、同 A* 输入下的基线及 `L₀/T₀`。
默认合并间隔不超过 1 m 的转弯，向两端直线各扩展最多 0.30 m。
按转弯事件划分后均匀布置节点，避免余下极短的最后一段。

分段五次 Bézier 共享位置、切线、曲率，构造 G2 连接。内部节点可移动，
同向弯不强制曲率归零，S 弯可自然过零。Ceres 使用分段局部参数块与三个确定性初值。
目标包含曲率、变化率、连接变化率差、速度不足、偏移和净空；区间上界惩罚防止优化器
利用积分采样点之间的尖峰。默认空构建类型对数值核心启用 `-O2`，显式 Debug/Release
保持所选编译配置。

连续校核使用 Bernstein 导数界、递归细分及控制点凸包包络，同时检查真实曲线、
离散弦线、偏移、正则性、采样航向、自交和窗口外部相交。默认偏移上限 0.20 m，
曲率上限 `1/arc_min_radius`，新曲线变化率上限 30 m⁻²。整条路径的长度和预计时间
分别不超过冻结基线的 110%。失败保留基线；回退段不承诺满足新曲线的变化率上限。

动态预算使用 `|a|K + v²Q ≤ α_ff`，取消固定对半分配；名义规划额外乘以
`dynamic_budget_utilization`。控制器以指令速度差计算前馈线加速度，用测量速度约束
曲率进度需求，并保留原角速度、角加速度和安全门。预测速度包络可以提前考虑预瞄，
在线需求校核使用原始几何区间，避免重复预瞄。

速度包络在 `v²` 上双向传播，包含停车点和区间内部速度峰。动态传播失败回退旧保守
预算，控制器遇到不可行状态仍停车。主动协调先为局部可行上限至少达到 0.20 m/s 的连续转弯区域尝试稳定平台，并独立校核
时间预算和加减速事件；不把短瓶颈所在的整个窗口机械压到最低速度。平台不包含窗口
两端直线扩展区，其他区域再合并相邻速度峰。协调不跨停车点，不强制
弯道最低速度，不压低足够长直线的巡航速度。0.20～0.25 m/s 是空间允许窗口的优化
目标，并非所有弯道都能达到。

## 构建与验证

依赖 Ubuntu/ROS 2 Humble、Ceres 2.0 或兼容版本；`plan` 声明 `ceres-solver` 依赖。
不要混用其他实验 overlay 的二进制与本次 YAML。

```bash
source /opt/ros/humble/setup.bash
colcon build --build-base build/bezier_window \
  --install-base install/bezier_window \
  --packages-up-to controller car_description --cmake-args -DBUILD_TESTING=ON
source install/bezier_window/setup.bash
colcon test --build-base build/bezier_window \
  --install-base install/bezier_window --packages-select plan controller \
  --ctest-args --output-on-failure
colcon test-result --test-result-base build/bezier_window --verbose
```

单元测试覆盖联合预算、零速端点、G2、八种开关组合、窄通道及校核预算耗尽。
控制器测试包含非线性 S 弯和两组 Gazebo 速度/位姿回放，防止重复预瞄及速度滞后导致
错误预算保护。回放位姿来自 TF 定位估计，不是 Gazebo 真值。

已有节点测试的速度、停车距离夹具显式固定，避免部署 YAML 调参改变其测试语义；
生产 YAML 的物理值没有因此改变。全包检查若报告既有 DWA 格式问题，应与本功能测试
区分，不能通过修改无关 DWA 文件消除报告。

## 可复现实验

```bash
# 固定地图及六组消融；synthetic 保留指定转弯，不做 shortcut。
python3 tools/run_bezier_benchmark.py --synthetic
python3 tools/run_bezier_benchmark.py

# 单独的 Gazebo 域和 master；启动前确认没有其他实验占用 YAML 中的端口。
# 驱动在所有试验前自动冻结离线窗口，必须传入上一步地图离线目录。
python3 tools/run_bezier_gazebo.py \
  --offline-dir log/bezier_window/具体时间_map

# 所有实验完成后生成 comparison.json 及每次 tracking_summary.json。
python3 tools/analyze_bezier_results.py \
  --gazebo-dir log/bezier_window/具体时间_gazebo
```

Gazebo 驱动使用现有 `gz model` 重置位姿，不添加传感器或插件。初始位置统一取请求
位置所在栅格中心，避免 0.05 m 栅格边界附近微小定位噪声造成不同 A* 起始单元；
报告同时保留请求位置、初始化目标和 TF 确认位置。起始朝向由实验 YAML 固定。
开发探针 `--variants combined --limit 1` 不能作为三次双向 A/B 验收。

记录保存压缩膨胀地图、SHA256、源码版本和差异、实际参数及算法诊断。
planner 的网格指纹与记录快照必须一致。`windows` 包含选中节点、区间曲率上界、
瓶颈速度、10%～90% 曲率过渡距离、求解/校核预算及拒绝原因；旧 `corners` 是冻结的
原有选弯搜索诊断，不能用其候选编号解释新的 Bézier 窗口。

验收同时检查目标到达、地图一致性、固定窗口速度平台、显著加减速减少 30%、
时间中位数不超过基线 110%、横向 P95 增量和意外停车。jerk、参考/指令/测量速度分别
统计，缺口和状态切换不跨越。任一指标未通过仍保留实验状态，不切换默认。

## 本轮结果

完整统计及未达门槛项见[验证结果](bezier_window_validation.md)。默认实验开关仍关闭。
动态预算要求 planner 启用曲率连续过渡；旧模式不能用不连续曲率标志隐藏普通节点间
的曲率变化。回放夹具存放于 `src/controller/test/fixtures/`，可直接参与版本审查。
