# P1：Bézier 残余曲率尖峰优化与验证

日期：2026-10-30（Asia/Shanghai）。本轮起点为 `lqr` 分支的 `80e3a1605d48c55cb8f2ea4388c778b89ebfd22e`。本轮修改尚未提交。

**当前结论：E3 按下述未修改场景波动口径通过离线验证；闭环因原控制组的启动／终点到达问题中断，完整 P1 未通过。新增策略默认关闭。** 每窗口求解预算仍为 0.25 s，整个 Bézier 处理预算仍为 1.0 s，最多 50 次迭代。动态角加速度预算与整段速度协调均关闭。

## 诊断与输入证据

主要记录为 `data/path_record/run_20261030_142246_620464`。历史发布路径约 13.485 m 处节点曲率为 2.36146 m⁻¹，后段 Bézier 已被采用。独立重建的同条件 A 回放中，密集解析峰值约 2.36347 m⁻¹，落在末段内部 `t≈0.1953`；连接曲率差最大约 3.52×10⁻¹³。因此，本例尖峰属于段内几何，不是已证实的 G2 连接错误。历史记录未保存控制点，不把回放控制点称作历史原始控制点。

输入冻结在 `log/bezier_refine/frozen/inputs/`：

- `latest` 从历史已膨胀栅格、捕获参数和端点重建 A*，先关闭 Bézier，保存原始 A* 与进入 Bézier 前的安全路径。各组随后只读取这一份安全路径，不再重复 shortcut、膨胀或累积平滑偏移。
- `old` 沿用 P0 已证明零窗口采用的旧冻结安全路径，保留原 P1 的固定区域及门槛。
- 四个合成场景分别冻结安全输入：直线、直角、同向双弯、S 弯。

历史发布进度区间的端点先按空间位置投影到冻结安全路径，之后固定评价范围。最新记录的三个参考范围约为 `[1.3000, 4.4272]`、`[7.4272, 11.3921]`、`[12.2866, 13.8418]` m。投影误差单列在 `regions.json`，最大约 0.134 m；不将两条不同路径上的相同里程直接等同。

## 实现与实验选择

核心仍是五次 Bézier 共享几何节点。新功能位于原几何核心，ROS 消息及控制器接口没有改变：

1. 峰值敏感目标：分离超阈值曲率积分权重，并用共享辅助变量惩罚窗口采样最大曲率。辅助变量是软目标，不是连续最大曲率证书；最终仍独立验证整条曲线、正则性和输出弦线。
2. 评价一致性：基线和候选使用相同数量的均匀弧长评价点，不再将基线离散节点连接代价与候选解析连接代价直接混比。优化器曲率约束参考值与最终校核的原窗口峰值上限对齐。
3. 搜索效率：优先尝试尺度接近 1.0 的粗初值，其首轮份额为父窗口剩余预算的 70%；保留父子窗口分配边界。缓存重复使用的精确整数二项式系数，减少证书残差计算。分别记录拟合、初值检查、构建残差、求解、校核和评分耗时。
4. 局部精修：已采用曲线仍超过目标曲率时，最多尝试一次峰值段及其相邻段的联合精修。窗口外节点及局部外端完整固定，整个候选仍相对原安全路径校核。新峰值上界、变化率上界或变化率积分增大时拒绝精修。
5. 有限启用范围：实验发现全窗口统一改目标会损害较简单弯道。E3 仅对原峰值大于 2.0 m⁻¹、至少包含 3 个既有曲率事件的窗口启用新策略；其余窗口使用原策略。这里的“事件”是现有 `events()` 的曲率区间定义，不等于人工统计的物理弯道数量。

实验依次保留在 A、B1/B2/B4、C 系列、D 系列及 E 系列目录。降低曲率但提高变化率的早期组未被选用。最终 E3 的关键覆盖值如下；省略新参数时沿用旧行为，主 `astar.yaml` 未切换到 E3：

```yaml
window_refine_objective: true
window_max_curvature_weight: 1.0
window_peak_weight: 1.0
window_prioritize_coarse_seed: true
window_coarse_budget_fraction: 0.7
window_refine_rate_weight: 4.0
window_refine_min_curvature: 2.0
window_refine_min_events: 3
window_local_refinement: true
```

原 `window_rate_weight` 保留，用于不满足触发条件的旧策略；`window_refine_rate_weight=-1` 表示继承它。新策略不改 `motion_limits.yaml`、速度包络、控制器、消息或 URDF。

## 离线结果及验收口径

各场景各组运行 5 次，不选择性排除较差运行。表中 K/Q 为固定区域的区间可信上界，RMS 为发布节点曲率按弧长计算的变化率 RMS；密集解析曲率/变化率另存，二者不混称。

| 场景／版本 | 后段 K / m⁻¹ | 后段 Q / m⁻² | 后段变化率 RMS / m⁻² | 发布长度 / m | 预计时间 / s | 最长离线处理 / s |
|---|---:|---:|---:|---:|---:|---:|
| latest 原始 A | 2.3770 | 19.2263 | 5.4318 | 16.4886 | 46.9065 | 0.5643 |
| latest 缓存后控制 Afinal | 1.7631 | 11.1267 | 3.8424 | 16.4659 | 48.6453 | 0.5646 |
| latest E3 | 1.3642 | 6.4108 | 1.7600 | 16.3961 | 43.6464 | 0.5323 |
| old 原始 A | 2.5000 | 62.5064 | 20.1896 | 16.3225 | 47.0681 | 0.6938 |
| old 缓存后控制 Afinal | 2.1386 | 15.1332 | 3.7057 | 16.5829 | 45.6495 | 0.5995 |
| old E3 | 1.4879 | 6.3240 | 1.6076 | 16.4638 | 43.6887 | 0.5698 |

K/Q/RMS 和耗时取五次最差值，长度及预计时间列为第一重复的对应值。预计时间来自原速度核心，不是 Gazebo 通行时间。缓存使墙钟截止前能完成更多优化工作，因而同一数学目标也可能得到不同结果，所以同时保留原始 A 与同版本控制 Afinal；不能把全部收益归因于新增目标函数。

相对同版本 Afinal，latest 后段最差 K 下降约 22.6%、Q 下降约 42.4%、变化率 RMS 下降约 54.2%；相对原始 A 的 K 下降约 42.6%。旧回归后段满足原 `K≤2.25、Q≤30` 门槛。两条实图路线的所有固定区域均通过严格不增大检查。独立密集采样的最大偏移分别约 0.11670 m、0.09847 m；这两项是采样估计，完整曲线仍以 0.20 m 连续偏移校核为准，见 `sampled_deviation.json`。

**对未修改场景的墙钟波动单独处理。** 直角场景的一次默认预算求解少完成一次迭代，导致 K 从 1.10120 变为 1.10685（约 +0.51%）、变化率 RMS 从 0.81638 变为 0.82174（约 +0.66%）。严格逐值检查保留为失败；没有删除该运行。四个合成场景的固定工作量对照逐字段一致，且两组使用同一二进制，证明触发范围外没有算法差异。

分析工具默认仍用严格比较；本轮显式采用 `--unchanged-scope-tolerance .01`，仅当**全部窗口未启用新目标、有同一二进制的工作量等价证明、全部五次完整**时，允许最多 1% 的墙钟截断波动。启用新策略的区域不适用该容差。它不放宽安全、0.20 m 偏移、110% 长度／预计时间以及原 P1 的 K/Q 门槛。默认预算效果只来自五次默认预算运行；宽裕时间的对照仅证明代码等价，不作为优化收益。

每次回放保留逐候选理由、实际采用节点、重建控制点、解析密集样本、离散三点曲率、连接左右差和预算分项。局部精修还核对固定区域外的初始/最终节点逐字段一致。配置预算不是硬实时保证，单次不可中断迭代的超时量独立记录。

主要证据：

- [离线验收 JSON](../log/bezier_refine/frozen/acceptance_E3.json)、[全部分析结果](../log/bezier_refine/frozen/analysis.json)。
- [新路线对比图](../log/bezier_refine/frozen/comparison_latest.png)、[旧回归对比图](../log/bezier_refine/frozen/comparison_old.png)。
- [固定工作量与旧二进制兼容证明](../log/bezier_refine/frozen/scope_equivalence.json)：旧程序与新程序关闭新选项后，237 点全部字段差为零。
- `log/bezier_refine/before/` 保存本轮前 141 个源码文件摘要及原程序；每组另有源码归档、差异、程序 SHA256、覆盖参数和实际执行命令。`log/` 被 git 忽略，分享报告时须同时保留这些产物。

## 构建、测试和闭环验证

构建成功。当前 205 项功能测试通过（plan 104、controller 101），包含本轮新增的 4 项 C++ 和 7 项分析测试。新增测试覆盖解析变化率核对、采样间漏峰被连续证书拒绝、原安全路径偏移、局部精修、最差重复保留，以及未修改场景容差不能用于实际启用新策略的区域。

旧圆弧开关测试现在显式关闭窗口 Bézier，避免当前主 YAML 已开启的窗口功能污染该测试的独立断言。没有改变断言或生产主开关。

全包测试存在既有 DWA 格式及末尾换行失败；本轮不修改这些无关文件。沙箱内额外出现 XML schema 下载与 DDS 套接字限制，已在允许本机 ROS 通信及只读 schema 获取的环境中复跑。最新汇总与详细失败见 `log/bezier_refine/final_test_results_latest.log`；不能声称全包 lint 全通过。

已运行的主要命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --build-base build/bezier_refine --install-base install/bezier_refine \
  --packages-up-to controller car_description --cmake-args -DBUILD_TESTING=ON
source install/bezier_refine/setup.bash
colcon test --build-base build/bezier_refine --install-base install/bezier_refine \
  --packages-select plan controller --ctest-args --output-on-failure
colcon test-result --test-result-base build/bezier_refine --verbose

# 新实验使用新目录，勿覆盖已有冻结数据。
python3 tools/run_bezier_refine.py --root log/bezier_refine/new_run \
  --variants Afinal D3 E3 --repeats 5
# 为未修改的合成场景生成与本组二进制匹配的固定工作量证明。
python3 tools/run_bezier_refine.py --root log/bezier_refine/new_run \
  --prove-unchanged-scope E3
# 对已归档数据生成图表和验收。
MPLCONFIGDIR="$PWD/log/bezier_refine/matplotlib" \
python3 tools/analyze_bezier_refine.py log/bezier_refine/frozen \
  --candidate E3 --baseline-variant Afinal --unchanged-scope-tolerance .01
```

闭环入口已支持 `current_bezier` 与 `optimized_bezier`，两组均关闭动态预算和速度协调，并使用最新冻结路线的正反向端点；通过离线验收后才能运行。未复用原 `combined` 三功能实验来声称本轮通过。闭环的到达、意外停车、时间中位数、横向 P95 增量以及最大横向误差均保留；横向 P95 仍沿用原 `max(2 mm, 10% 基线)` 增量门槛。

```bash
python3 tools/run_bezier_gazebo.py --offline-dir log/bezier_refine/frozen \
  --geometry-candidate E3
python3 tools/analyze_bezier_refine_gazebo.py log/bezier_refine_gazebo/具体目录
```

### 实际闭环结果：控制组前提未满足，停止后续对照

首次实验 `log/bezier_refine_gazebo/20261030_153457_418993_gazebo`：控制组去程三次均到达，通行时间分别为 53.99、54.16、53.96 s。首次回程在规划前启动失败：地图服务器生命周期调用超时，AMCL 持续等待地图，90 s 内未建立定位 TF。该无效启动完整保留。

使用既有一次无效启动重试额度，整组从头重启，记录在 `log/bezier_refine_gazebo/20261030_154104_152395_gazebo`。首个去程到达，时间 54.22 s；第二个去程规划成功，但在 300 s 墙钟超时时仍为 `TRACKING`，记录通行时间 299.57 ROS s，没有报告到达。

终点故障的最后观测值见[诊断 JSON](../log/bezier_refine_gazebo/20261030_154104_152395_gazebo/current_bezier_outward_2/terminal_diagnosis.json)：

- 距终点欧氏距离约 0.03042 m，大于配置的 0.030 m 到达门槛。
- 沿路径剩余距离约 0.01952 m，横向误差约 0.02333 m。
- `v_ref=0`、`v_cmd=0`，机器人长期停留在末段，尚未报告 `GOAL_REACHED`。

只读核对 `src/controller/src/lqr_continuous.cpp`：到达判定使用欧氏距离，而速度上限包含预瞄点的端点速度和制动距离。预瞄已到终点时，终点速度帽与剩余预瞄制动距离同时为零，可使速度上限先变为零。这与此次“尚在欧氏到达阈值外但不再前进”的观测一致；本轮没有修改控制器或放宽到达容差。

两批实验的全部有效／失败记录均保留。实验驱动在控制组前提失败后停止，**候选组未运行，正反向各三次 A/B 未完成，不能比较或宣称候选的实际跟踪误差、到达率和通行时间改善**。第二次部分记录的消息、参数、源码和地图关联检查通过，但总体比较因样本不完整及到达失败而未通过。详见[重试比较结果](../log/bezier_refine_gazebo/20261030_154104_152395_gazebo/geometry_tracking_comparison.json)。

后续前提是单独解决或确认控制组的地图生命周期可靠性及终点预瞄制动／到达一致性，然后以同一几何配置重新进行完整闭环验证。本轮止于 P1，保留实验状态，不进入后续速度优化阶段。
