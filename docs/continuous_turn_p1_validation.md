# P1：Bézier 候选可追溯与连续弯道几何优化结果

日期：2026-10-29（Asia/Shanghai）。实现基于 P0 的 `0835980a668b2ec17490e566621f3423fdb2c686`，本轮改动尚未提交。

**结论：P1 实现和离线验证已完成，但默认预算下的阶段验收未通过。** 前段与多个安全子窗口已经采用新曲线；后段最紧部分仍回退，固定区域的峰值曲率及变化率门槛未达到。扩大预算诊断找到通过校核的整体曲线，但不计入默认预算验收。因此没有启动本阶段 Gazebo 对比，也没有进入 P2，实验主开关继续默认关闭。

## 问题一：候选生成、校核、评分和最终采用是否可以追溯？

**答案：可以。新增 `geometry_first` 策略保留逐候选历史，搜索结束不再覆盖候选拒绝原因。旧 `legacy` 策略保留用于 A/B。**

每个根窗口、子窗口、候选均有编号。诊断记录初值类型、导数尺度、基线弧长、Ceres 终止名称／文本／时间预算、迭代记录数、实际求解耗时、校核次数、几何代价分量、固定速度规则下的事件数、独立校核结果、最终选择和父子关系。Ceres 的迭代记录包括初始项，不能把记录数 51 误读为超出配置的 50 次更新。

- `result` 表示候选／窗口处理结果；`search_end` 表示搜索为何结束，两者独立。
- `selected_candidate` 关联采用的候选；`child_adoption` 表示根窗口未采用整窗候选但子窗口有替换。
- `final_geometry_id` 是最终发布几何的 FNV 标识，包含位置、切线、曲率和停车标记，不包含速度帽。
- `geometry_reference_s` 将最终节点映射到不可变基线弧长，用于固定区域评价，避免路径长度改变后按相同数组下标比较。
- 完整控制点、失败参数区间及包络阻挡栅格由详细诊断开关控制；轻量候选历史始终保留。`geometry_fallback_reason` 记录后置全局回退，并保留此前历史。

代码依据：[新几何优化核心](../src/plan/src/path_geometry_optimizer.cpp)、[实验处理入口](../src/plan/src/path_experiments.cpp)、[诊断序列化](../src/plan/src/path_optimizer.cpp)。`nav_msgs/Path` 与 `PathProfile` 消息结构没有改变，诊断通过既有 JSON 字符串传递。

证据局限：几何标识用于关联，不替代碰撞证书；历史运行没有因此补出当时未记录的候选。

## 问题二：本次如何改善初值和几何选优，是否改了速度算法？

**答案：几何选优独立于速度功能；速度算法、物理参数和控制器源码保持原样。**

新策略以关闭动态预算及速度协调的固定规则做可行速度和时间门槛检查。优化与评分保留曲率、变化率、曲率软峰值、连接变化率差、偏移及净空项，取消速度不足项对新策略几何目标的影响。候选先满足安全、峰值不增加、变化率不超过 30 m⁻²、停车不增加和全局预算，再比较几何代价；同停车数时至少改善 1% 才首次替换。速度事件数只记录，不优先否决几何改善。

初值有两种，均会标明身份：

1. `baseline_fit`：覆盖曲率阶段边界和均匀节点，按几何最近距离自适应拟合，目标误差 5 mm、最多 64 个节点。该目标用于初值，不替代最终偏移和安全校核。
2. `coarse_spaced`：导数尺度最接近 1 的一个初值，另用约 0.5 m 的均匀节点，避免所有初值都继承 4 cm 过渡布局。它不宣称满足保形初值误差目标，必须经过完整优化和独立校核。

仍使用 0.75、1.0、1.25 三个导数尺度。共享节点改用**无量纲导数尺度**，各段以自身弦长构造端点导数，保留共享位置、切线和曲率，因此仍为 G2；不要求不等长片段具有相同的参数导数长度。旧策略的参数化不变。诊断里的 `derivative_scales_are_dimensionless` 明确区分两种含义，不能用旧策略的控制点公式解读新策略节点。

优化器内部使用连续插值的有符号距离软代价，以及块稀疏求解；它们不改变公共地图净空查询或安全判定。几何选定后仍调用原速度处理，几何核心在完整工作量测试中不受速度功能开关影响；墙钟截断情况下不承诺位级可重复。

本轮已核对 **77 个受保护的既有文件 SHA256**，包括控制器、URDF、消息包、共享物理限制及速度包络核心，均未改变。

另外用修改前冻结的二进制重跑相同原始 A* 输入，核对其原方法、旧 Bézier 与新版本对应输出：两组各 243 个节点，全部字段最大数值差约 7.11×10⁻¹⁵，仅为 CSV 输出精度差异。证据见 [旧二进制兼容核查](../log/bezier_p1/before/raw_chain/compatibility.json)。旧配置缺少 P1 参数的隔离域规划器启动检查也通过，见 `log/bezier_p1/legacy_yaml_smoke.log`。

## 问题三：碰撞拒绝是否仍只能看到一个 `collision`？子窗口是否安全？

**答案：新策略区分碰撞见证与安全性未证实，并支持有界的安全子窗口替换。**

包络接触障碍时继续细分曲线；曲线上的点明确落入阻挡区域时记录 `collision_witness`。达到深度或资源上限仍无法证明安全时记录 `collision_unproven`、`check_budget` 等结果并拒绝。诊断中的 `envelope_blocking_cell` 只是阻挡包络的栅格，不能当作真实曲线碰撞见证；`sample_witness` 是另外的字段。

最终采用仍要求整个连续曲线、输出弦线、正则性、曲率／变化率、偏移、自交、窗口外相交及全局预算检查通过。长度校核累计新曲线各叶片的控制多边形长度上界，表格中的发布弦线长度另行统计。

整窗失败后最多拆分两层，子窗口至少 1.0 m 且含两个转弯事件。优先利用已有直线间隔或较小变化率位置切分，边界匹配冻结基线的位姿、切线和曲率，允许非零曲率连接，不人为插入直线。父子共享原窗口预算，所有替换相对最初基线检查偏移。相邻子窗口拼接会保留已采用后邻片段的出段证书，避免左侧替换覆盖右侧限制。

新增测试覆盖：包络误拒绝后细分通过、端点安全但中部真实碰撞、校核预算耗尽、长短段 G2、速度开关独立性、非法配置，以及父窗口拒绝而安全子窗口被采用。

证据局限：拒绝未证实安全的候选不等于证明不存在可行曲线；也不以稀疏点无碰撞代替连续证明。

## 问题四：固定前段和后段实际改善了多少？

**答案：默认预算下前段明显改善，后段部分改善，但整个固定后段仍未通过门槛；扩大预算时找到整体改善的安全曲线。**

冻结输入来自 `run_20261029_184651_724167`：直接恢复已膨胀地图和已发布路径，回放窗口处理层，不二次膨胀，也不再次 shortcut。原方法、旧 Bézier、新 Bézier 三组的动态预算和速度协调均关闭。

| 指标 | 原方法／默认旧 Bézier | 新 Bézier 默认预算 | 新 Bézier 扩大预算诊断 |
| --- | ---: | ---: | ---: |
| 前段 K 上界 / m⁻¹ | 2.0000 | 1.3908 | 1.3447 |
| 前段 Q 上界 / m⁻² | 50.0033 | 6.3757 | 4.5632 |
| 后段 K 上界 / m⁻¹ | 2.5000 | 2.5000 | 1.7455 |
| 后段 Q 上界 / m⁻² | 62.5064 | 62.5064 | 9.8042 |
| 后段最小局部发布速度帽 / m/s | 0.1225 | 0.1225 | 0.2394 |
| 发布弦线总长 / m | 16.2680 | 16.3225 | 16.5745 |
| 现有速度核心预计时间 / s | 53.9178 | 47.0678 | 43.9606 |
| 新策略本次离线处理耗时 / s | — | 0.6749 | 2.3788 |

前段固定范围为基线 s=1.5～3.0 m，后段为 s=11.5～13.5 m；新方案按记录的基线对应关系评价全部跨区域区间，包含回退部分。表中 K/Q 是区间可信上界；图中的节点曲率另有标签，不将两者混用。

默认预算根窗口中，前窗、末窗采用整体曲线；中间大窗口通过两个子窗口替换了约 **7.4796～12.1088 m**，但约 **12.1088～13.6890 m** 仍保留基线。这正是固定后段最大 K/Q 未下降的原因，不能只统计成功子窗口后声称通过。

前段单调曲率上升区间的节点 10%～90% 距离约 0.537 m，主要转向变化不再集中在原有约 4 cm 过渡内。转向过零和局部小幅变化仍存在，不宣称所有弯道变成单一圆弧。CSV 中的单调幅值变化距离是节点估计，ROI 边界截断的变化不作为完整转弯长度证明。

表中速度帽是局部上限，预计时间不包含实测跟踪、定位等待和实际规划等待；它们都不是 Gazebo 实测通行速度／时间。不同阶段不共享这些量的物理含义。

原始产物与图：

- [默认预算回放及机器验收](../log/bezier_p1/20261029_214036_replay_default/p1_analysis.json)、[几何对比图](../log/bezier_p1/20261029_214036_replay_default/frozen_geometry_comparison.png)。
- [扩大预算诊断](../log/bezier_p1/20261029_214038_replay_expanded/p1_analysis.json)、[几何对比图](../log/bezier_p1/20261029_214038_replay_expanded/frozen_geometry_comparison.png)。
- [完整 A* 链路同条件对比](../log/bezier_p1/20261029_214044_map_default/p1_analysis.json)，固定目标为 `(-4.375, 7.645)`，保存原始 A* 路径及输入。
- [合成场景对比](../log/bezier_p1/20261029_214047_synthetic_default/p1_analysis.json)，覆盖直线、直角、同向连续弯和 S 弯。

各目录保存参数副本、原始输入摘要、源码快照、差异、执行命令、二进制 SHA256、最终路径、速度包络及逐候选报告。默认／扩大预算回放使用的规划基准程序二进制 SHA256 相同，均为 `5cf68cf5d1ac21c8464d0ab1d4b9ad71e09b25bcee51c8361aa9a66920c1f5e6`。分析结果另记录分析脚本摘要。

## 问题五：现在的主要剩余原因是什么，是否应立即提高车速或物理限值？

**答案：现有证据指向默认预算内的候选优化尚不充分，不支持先提高物理限值。扩大预算已找到可校核的整体改善候选，但不是默认预算成功。**

默认大窗口的粗初值只完成约 5 条迭代记录，Ceres 明确报告达到约 0.041 s 的分配时间。该候选失败子段的解析采样曲率约 2.6499 m⁻¹，超过被替换窗口 2.5 m⁻¹ 的峰值门槛，因而拒绝；其他保形候选也有曲率或变化率超限。

扩大预算时，相同粗初值完成约 31 条迭代记录，求解约 0.323 s，完整窗口通过校核和几何选优。固定后段 K 降低约 30.2%，Q 降至约 9.80 m⁻²。这说明本次较大预算搜索已找到满足几何门槛的候选，不能继续把整个区域判定为几何不可行；也不能证明默认预算内绝对无法找到其他更好的候选。

下一次 P1 改进应聚焦相同预算下的搜索效率、初值质量及预算分配，并保留本次冻结输入作回归。不能把诊断的 2 s／窗口、8 s／路径直接改成默认，也不能用速度功能放宽物理约束来替代几何优化。

## 问题六：如何调试和复现？

**答案：新增调参入口集中在 YAML，旧配置缺少新参数时仍选择旧策略。**

[astar.yaml](../src/plan/config/astar.yaml) 中 `planner.ros__parameters.post_processing` 的参数：

| 参数 | 当前值与作用 |
| --- | --- |
| `enable_window_bezier` | false，主实验开关仍关闭 |
| `window_geometry_policy` | `geometry_first`；指定 `legacy` 对照旧策略；核心兼容默认值为 `legacy` |
| `window_detailed_diagnostics` | false；离线驱动显式开启，保存控制点和失败叶片 |
| `window_include_coarse_seed` | true，增加不同节点布局的粗初值 |
| `window_seed_fit_tolerance` | 0.005 m，保形初值拟合目标 |
| `window_geometry_min_gain` | 0.01，首次替换要求几何代价至少改善 1% |
| `window_parent_budget_fraction` | 0.5，可拆分窗口给整窗尝试的份额 |
| `window_split_max_depth` | 2，最大拆分深度 |
| `window_split_min_length` / `window_split_min_events` | 1.0 m／2 个事件，子窗口门槛 |
| `window_solver_seconds` / `window_total_seconds` | 0.25 s／1 s，保持原默认值 |

场景、固定区域、扩大预算和验收阈值位于 [bezier_benchmark.yaml](../src/plan/config/bezier_benchmark.yaml) 的 `benchmark.p1` 下。物理参数唯一来源仍为原共享 YAML。

```bash
source /opt/ros/humble/setup.bash
source install/bezier_p1/setup.bash
python3 tools/run_bezier_p1.py --mode replay --budget default
python3 tools/run_bezier_p1.py --mode replay --budget expanded
python3 tools/run_bezier_p1.py --mode map --budget default
python3 tools/run_bezier_p1.py --mode synthetic --budget default

export MPLCONFIGDIR="$PWD/log/bezier_p1/matplotlib"
python3 tools/analyze_bezier_p1.py log/bezier_p1/具体回放目录
```

扩大预算仅使用输出目录中的参数副本。普通功能测试未被改成用扩大预算验收，主 YAML 的预算没有提高。

## 问题七：构建和测试是否通过，是否实际跑了仿真？

**答案：构建成功，194 项功能测试通过；全包 lint 仍有既有 DWA 问题。本轮没有运行 Gazebo。**

实际完成 plan 93 项、controller 101 项功能测试，其中新增 6 项 GeometryFirst 测试。修改涉及的 C++ 格式检查及三个 Python 工具的 flake8 检查通过。

全包 `colcon test-result` 为 **323 tests、0 errors、6 failures、38 skipped**；失败来自未修改的 DWA 头文件／两个源文件格式和 DWA launch 文件末尾换行，包含 CTest 包装失败项，不能声称所有检查均通过。没有修改无关 DWA 文件。

构建／回归命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --build-base build/bezier_p1 --install-base install/bezier_p1 \
  --packages-up-to controller car_description --cmake-args -DBUILD_TESTING=ON
source install/bezier_p1/setup.bash
colcon test --build-base build/bezier_p1 --install-base install/bezier_p1 \
  --packages-select plan controller --ctest-args --output-on-failure
colcon test-result --test-result-base build/bezier_p1 --verbose
```

首次全链路构建已成功；最后仅诊断输出修改后的成功构建使用同一 overlay 的 `--packages-select plan controller car_description`，日志为 `log/bezier_p1/final_build3.log`。测试日志为 `final_tests.log`、`final_test_results.log`；两者均在该目录中。

## 问题八：P1 是否通过，可以进入 P2 吗？

**答案：不可以。P1 默认预算验收未通过，应继续停留在 P1。**

| 门槛 | 默认预算结果 |
| --- | --- |
| 前段实际采用且有几何改善 | 通过 |
| 固定后段 K≤2.25 m⁻¹ | 未通过，仍为 2.5 |
| 固定后段 Q≤30 m⁻² | 未通过，仍为 62.5064 |
| 已采用新曲线的独立校核、长度与预计时间约束 | 通过 |
| 固定路线实际跟踪误差、时间及到达验证 | 未执行；离线门槛未过 |
| 本阶段总体结论 | 保留实验状态，不进入 P2 |

扩大预算诊断单列为几何可行候选证据，不计入默认采用率。当前结果不能用于宣称实际横向误差改善、稳定车速达到目标或三组仿真通过；后续仍须沿用既有跟踪误差门槛，不能降低验收标准。

本次实现文件保持工作区修改状态；保留原有 P0 文档的暂存状态，不暂存本次 P1 实现、不提交或推送。
