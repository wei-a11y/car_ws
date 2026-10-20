# 减少转弯停车：联合选弯、局部改线与速度协同

## 数据流和调试入口

保持 `Goal -> A* -> Path Processing -> 纵向 PID / 横向及航向 LQR -> Safety Gate -> /cmd_vel`。
`/plan` 仍为 `nav_msgs/Path`，发布最终通过校核的执行曲线。控制器、记录器及原有绘图共用这条参考。
本轮不调整巡航速度、PID 参数、LQR 权重、地图膨胀距离、绘图算法或默认记录目录选择逻辑。

| 文件 | 用途 |
| --- | --- |
| `src/plan/config/motion_limits.yaml` | planner 和 LQR 共用的巡航/最高速度、角速度、加减速度、横向加速度、前馈预算及最低可行过弯速度 |
| `src/plan/config/astar.yaml` | 半径/过渡长度搜索、相邻弯候选及 DP 标签预算、局部改线、回退整形精度和代价 |
| `src/controller/config/lqr_tracker.yaml` | PID、LQR、预瞄、跟踪和数据有效性约束 |
| `src/controller/config/path_tracking.yaml` | 记录及现有三幅图的分析参数 |

运动参数的唯一配置来源改为 `motion_limits.yaml`。导航、独立 planner、独立 LQR 的 launch 均加载各节点参数、部署覆盖文件后，再加载该共享文件。
共享通配参数文件放在最后，避免 ROS 2 Humble 的参数合并顺序使命令行 `-p` 覆盖失效。
参数在启动时读取，修改后重启。直接运行可执行程序也必须在节点参数文件之后传 `--params-file src/plan/config/motion_limits.yaml`。
巡航仍是 **0.15 m/s**，后续提速时在共享文件同时调整 `v_nominal/v_max`，并重新验证 Safety Gate、底盘和跟踪误差。
`min_turn_speed: 0.03` 是不停候选的可行性门槛，不会强制起步、终点及必要停车制动时保持该速度；实际执行约束可以进一步降速。

## 搜索与安全校核

每个弯使用两侧完整线段，生成停车折角、连续曲率过渡和整形圆弧候选。连续过渡联合搜索半径与过渡长度，小转角允许只有双过渡段，等效半径可以超出圆弧搜索上限。
链式 DP 检查相邻候选的出口/入口裁剪长度之和不超过共享线段，同时保留考虑长度的 Pareto 标签。
排序为停车数最少、圆弧回退数最少，再比较预计时间和平滑/净空代价。原路线、shortcut 及局部改线路线共用评价标准。
预计时间包含起步、制动、必要停车转向；DP 标签中的时间是有限候选排序的近似代价，不是对真实闭环通行时间的保证。

局部改线优先处理保留停车点及相邻弯，在局部窗口的固定端点之间平移走廊锚点，再由同一曲线优化器连接。
默认单次改线最大 0.20 m，窗口为折角前后各 2 m。每个新线段和曲线均重新检查碰撞；总路径长度不超过本次查询旧安全基线路径的 **110%**。
该改线预算与圆弧回退的 1 cm 整形预算是两种独立约束。两轮局部候选搜索受到路线数、标签数及工作量限制，不保证全局最优。

`max_curvature_rate` 在联合模式中用于舒适性代价，实际可行性遵守：

```text
omega = v * kappa
alpha = a * kappa + v^2 * kappa_prime
```

前馈角加速度预算各一半留给两项，另保留反馈余量；同时限制角速度及横向加速度。
空间紧张时通过降低可行速度保留候选；速度上限小于 0.03 m/s 则拒绝不停候选。
控制器将规划上限与自身执行约束取最小值，建立包含所有中间限速点的制动包络，然后运行 PID 和角速度前馈。
旧算法仅用于安全初始解，仍保留旧的几何阈值；若其几何曲线不满足新的共享运动约束，则降为已校核弦线上的停车/对准参考。

普通圆弧回退使用五次 Hermite 入口/出口，匹配位置、切线及端点曲率。
通过 Bernstein 导数界和递归细分检查整段的非零切向、曲率、曲率变化率、速度及碰撞包络，并证明整形偏移不超过 0.01 m。
同时检查原圆弧及连接直线的 1 cm 包络，保留原地图膨胀。回旋线的碰撞包络还包含弦偏差和数值积分误差。
不会通过取消停车标记强行通过不可行折角。

工作量耗尽时返回已找到的最佳安全解；若尚未完成任何联合候选，则保留初始安全解。
标签超限时保留最优标签和最短标签并标记搜索受限，继续尝试其他路线。`budget_exhausted` 表示搜索受限，不能据此声称结果最优。

## 接口和记录

`PathProfile` 的旧字段保持语义，扩展以下出段字段（索引 i 对应 i 到 i+1）：

- `segment_type`：0 直线、1 连续曲率过渡、2 整形圆弧回退。
- `speed_limit`：出段局部速度上限，末项为 0，不表示新出段。
- `curvature_bound`、`curvature_rate_bound`：整段的绝对值上界；控制器仍会按自己的离散参考核对并收紧。
- `processing_report`：JSON，包含长度、预计时间、搜索预算、保留停车点及逐候选参数/拒绝原因。`chosen_candidate` 是可行候选表的索引，0 表示停车，其他索引对应 trials 中依次通过的候选，null 表示保留旧安全初始解。

四个数组要么全部为空（兼容旧生产者），要么全部与路径点等长。控制器拒绝部分数组、非法数值和普通点的零限速。
记录器在 `planned_path.csv` 保存新增数组，并在每个路径 epoch 保存 `planning_report.json`。
诊断区分空间、碰撞、曲率、最低速度、整形偏移及相邻空间冲突/长度预算。预算耗尽单独在报告顶层记录，`budget_reason` 区分 work_limit、label_limit、candidate_limit、sampling_limit、output_limit。

## 构建、启动及验证

消息已改变，必须构建并重启 `common_msg`、`plan`、`controller`。导航 launch 的改动也需安装 `car_description`。

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/joint_turn build \
  --build-base build/joint_turn --install-base install/joint_turn \
  --packages-select common_msg plan controller car_description \
  --cmake-args -DBUILD_TESTING=ON
source install/joint_turn/setup.bash
ros2 launch controller path_tracking.launch.py \
  params_file:="$PWD/src/controller/config/path_tracking.yaml"
```

另一个使用同一 overlay 的终端：

```bash
ros2 launch car_description navigation.launch.py controller_type:=lqr \
  motion_limits_file:="$PWD/src/plan/config/motion_limits.yaml" \
  planner_params_file:="$PWD/src/plan/config/astar.yaml" \
  lqr_params_file:="$PWD/src/controller/config/lqr_tracker.yaml"
```

相关测试：

```bash
colcon --log-base log/joint_turn_test test \
  --build-base build/joint_turn --install-base install/joint_turn \
  --packages-select plan controller \
  --ctest-args -R 'test_planning_grid$|test_astar$|test_path_processing$|test_joint_turns$|test_lqr_tracker$|test_continuous_tracking$|test_astar_node$|test_lqr_tracker_node$|test_path_recorder$|test_tracking_analysis$' \
  --output-on-failure
colcon test-result --test-result-base build/joint_turn --verbose
```

`test_joint_turns` 覆盖同向弯/S 弯联合分配、小转角、速度门槛、回退曲线独立密集细分、碰撞和预算回退、局部改线及长度预算。
跟踪回归使用 0.15 和 0.35 m/s、带速度滞后的合成模型，不能替代 Gazebo 实跑。
本次验证结果和环境限制见本文后续记录。所有代码保留为未提交修改，不暂存、不本地提交、不推送。

## 2026-10-20 验证记录

构建使用本文的 `build/joint_turn` / `install/joint_turn` 隔离 overlay，`common_msg`、`plan`、`controller`、`car_description` 构建成功。
后续增量修改还执行了 `cmake --build build/joint_turn/plan -j4`、`cmake --install build/joint_turn/plan`，以及 controller 对应命令。
相关 C++/ROS/记录器/分析器回归通过；`ament_uncrustify`、修改过的 Python 文件 `ament_flake8` 及 `git diff --check` 通过。
早期测试曾因沙箱禁止 DDS socket、共享 YAML 顺序以及测试缺少起步对准而失败，已定位并在修复后重新验证。

### 合成模型与地图离线验证

- 同向连续弯及 S 弯：共享直线 0.45 m 的样例，旧方法保留 1 处停车，联合选择为 0；不需要圆弧回退。
- 小转角双过渡、最低速度拒绝、极小整形偏移拒绝、碰撞/预算回退均有回归。
- 五次回退连接额外使用独立幂基重建及每段 2000 次细分检查，覆盖曲线内部的偏移与碰撞。
- 0.15 / 0.35 m/s 及速度滞后采用二维非线性差速模型，正常弯道持续 TRACKING，不能称为 Gazebo 的高速验证。
- 地图 `map/edited/edited.yaml` 的独立 A* 离线检查：旧基线 2 处停车，新方法 0 处；本次局部改线后长度约增加 0.94%，未超 10% 预算。

### Gazebo 实跑

使用仓库现有 world、地图、AMCL、Safety Gate，`controller_type:=lqr gui:=false rviz:=false`，巡航保持 0.15 m/s。
初始 Gazebo/AMCL 配置未调整，目标为 map 中 `(-0.875, 7.645)`。
新记录与旧记录 `run_20261019_203326_360761/path_0001` 的规划起终点完全一致，实际起点只存在微米级仿真漂移。
起点航向仍需先对准，终点仍正常制动停车。

完整实跑记录位于：
`data/path_record/joint_validation/run_20261020_190634_592737/path_0001/`。
其 `planned_path.csv`、`actual_path.csv`、`planning_report.json`、`tracking_summary.json` 及 `tracking_overview.png` 保留原始参考、实际 TF 跟踪及三幅图。
运行事件在上层 `events.csv`，验证状态在 `log/joint_gazebo_result.json`，验证脚本副本为 `log/joint_gazebo_monitor.py`。
`data/`、`log/` 按已有 `.gitignore` 保存为本地验证产物。

| 项目 | 结果 |
| --- | --- |
| 旧记录 / 新规划的中途必要停车 | 2 / 0 |
| 实跑非必要停车 | 0 |
| 实跑结果 | GOAL_REACHED，CSV 终点状态和事件均已记录 |
| 跟踪过程 | 起点 ALIGNING → 全程 TRACKING → 终点 BRAKING → GOAL_REACHED，中途无再次对准 |
| 新几何基线 / 执行路径长度 | 10.62109 / 10.61891 m |
| 横向误差 RMSE / P95 / 全阶段最大值 | 0.00820 / 0.01540 / 0.02819 m |
| TRACKING 阶段最大横向 / 航向误差 | 0.02075 m / 0.04844 rad（约 2.78°） |
| 曲线区横向误差 RMSE / 最大值 | 0.00704 / 0.02075 m |
| 参考进度覆盖率 | 99.74%，终点误差在现有到达容差内，不强制覆盖率为 100% |

该实跑按 TF 定位估计评估，未用 Gazebo ground truth 代替定位数据。
原绘图/统计包含起点对准，整体航向最大值约 2.614 rad；这与 TRACKING 阶段指标不同，未修改绘图来隐藏它。
搜索使用完 2,000,000 次工作量预算后保留零中途停车的安全解，没有证明通行时间全局最优。
运行期规划耗时约 5.7 s（本次未启用 Release 优化），工作量预算不是硬实时期限；需要更低延迟时应先用 Release/RelWithDebInfo 构建复测，再调整候选和搜索预算。

用原有分析器复现三幅图：

```bash
python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml \
  --data-dir data/path_record/joint_validation/run_20261020_190634_592737/path_0001 \
  --no-show
```

首轮 `run_20261020_190350_342737` 的状态监测也确认到达目标，但验证脚本提前关闭记录器，CSV 少收终点样本，不能用该 CSV 的 `goal_reached: false` 否定监测到的完成状态。
已重新实跑并使用上述第二轮完整记录作为本轮交付依据。
