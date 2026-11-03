# 贝塞尔几何与速度协调优化

## 阶段 0：冻结基准与显式双功能入口

主路线为 `run_20261031_202141_283365` 的已膨胀地图和端点。进入贝塞尔前的安全路径由同版本程序重建，不称为历史原始中间结果。动态预算通过实验覆盖关闭，静态约束保留，工作区 `motion_limits.yaml` 未修改。

`bezier_benchmark` 新增兼容关闭的 `benchmark_use_config_switches`，显式使用配置中的功能开关；`benchmark_speed_only` 要求冻结 profile，并核对输出几何标识。原 benchmark 组合语义保持兼容。每段限速来源和加减速能力保存到 `_limits.csv`，其中包络端点速度与局部几何速度帽分别列出。

实验配置：`src/plan/config/joint_bezier.yaml`。驱动：`tools/run_joint_bezier.py`。分析：`tools/analyze_joint_bezier.py`。A0/A 使用同一份冻结几何；连续证书、停车语义、位置、切线、曲率逐字段核对。主验收区在候选实验前选择基准最长反向弯根窗口，其他根窗口保护不退化；米数不进入生产策略。

产物：`log/joint_bezier/frozen_local/`，包含输入及二进制哈希、源码快照、逐候选诊断、最终节点、重建控制点、解析密集采样和连续证书。几何后续尝试另保存可执行文件副本。日志被 git 忽略，分享报告时需同时保存。最初沙箱 DDS 初始化失败位于 `log/joint_bezier/frozen/`，不计入有效样本。

有效基准含 8 场景各 5 次，包括主路线、历史回归、直线、直角、同向双弯、S 弯、短间隔及长间隔 S 弯。40 对 A0/A 几何全部一致。主路线未协调预计时间 44.521–44.541 s，当前协调 44.670–44.701 s，事件数由 17–18 降至 16。预计时间不是 Gazebo 通行时间。

已运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --build-base build/joint_bezier --install-base install/joint_bezier \
  --packages-up-to controller car_description --cmake-args -DBUILD_TESTING=ON
source install/joint_bezier/setup.bash
python3 tools/run_joint_bezier.py --root log/joint_bezier/frozen_local --stage baseline
python3 tools/analyze_joint_bezier.py log/joint_bezier/frozen_local --stage baseline
ctest --test-dir build/joint_bezier/plan \
  -R '^test_(planning_grid|astar|path_processing|joint_turns|window_bezier|bezier_refine_analysis|joint_bezier_analysis)$' \
  --output-on-failure
python3 -m pytest -q src/plan/test/test_joint_bezier_analysis.py \
  src/plan/test/test_bezier_refine_analysis.py
```

构建成功，7 个选定 CTest 目标通过，10 项 Python 分析测试通过。尚未宣称完整 ROS 节点、全包 lint 或闭环验收通过。本机 DDS 运行需要允许本机套接字；驱动为新执行调用增加 30 s 超时，保留失败日志。

后续按几何、速度、联合验证顺序推进；几何阶段未通过时不进入速度策略和闭环候选比较。

## 阶段 1：密集弯组几何候选离线通过

最终候选为 `attempts/dense/`：8 场景各 5 次默认预算运行，全部通过严格验收。每组使用同一二进制的控制组，不使用阶段 0 的旧二进制结果归因。开启条件为冻结安全窗口原峰值大于 2 m⁻¹、至少 4 个持久物理转弯；其余窗口保留现有算法。新增 `window_feature_refinement` 默认关闭，尚未进行候选闭环验收。

实现包含曲率滞回和最小持续弧长、跨停车点隔离、真实过零位置、特征节点初值、区间界引导的额外目标采样、多段热点联合精修，以及弯组 K/Q/RMS 不退化检查。先保留当前粗初值的有效候选，再用剩余父窗口预算精修；父窗口采用后不会执行子窗口回退，其预留份额可在原分配内复用。曲率界的固定数组计算只用于实验残差及峰值定位，最终连续安全证书仍走原校核。G2 由共享节点及固定外边界保持。

| 主路线主弯组，五次最差 | 同版本当前策略 | dense 候选 | 变化 |
|---|---:|---:|---:|
| 曲率可信上界 K / m⁻¹ | 1.97490 | 1.51535 | −23.3% |
| 变化率可信上界 Q / m⁻² | 27.25721 | 7.86098 | −71.2% |
| 节点变化率 RMS / m⁻² | 1.88364 | 1.27321 | −32.4% |
| 整条路线预计时间 / s | 44.5579 | 43.3376 | −2.74% |

前后保护窗口的最差 K/Q/RMS 相等。历史路线中段 K 从 1.95290 降至 1.34312，变化率 RMS 从 2.06869 降至 1.35784，Q 不增加。所有失败尝试保留：初轮小区域约束导致回退；`feature_final` 复跑未通过；无条件数组优化的 `feature_array` 未通过且未保留无条件优化；`feature_fast` 的短 S 弯 K 恶化约 2.4%，因此最终按物理复杂度限制新策略。每轮全部五次均保留，未放宽任何验收阈值。

固定工作量证明位于 `log/joint_bezier/scope_dense/scope_equivalence.json`：所有未触发场景的几何、证书和速度帽逐字段一致。宽裕时间只用于证明作用域，不计入收益。正式结果与图位于：

- `log/joint_bezier/frozen_local/attempts/dense/geometry_acceptance.json`
- `log/joint_bezier/frozen_local/attempts/dense/geometry_analysis.json`
- `log/joint_bezier/frozen_local/attempts/dense/geometry_comparison_latest.png`
- `log/joint_bezier/frozen_local/attempts/dense/geometry_comparison_old.png`

复现命令（输出目录不能覆盖旧产物）：

```bash
source /opt/ros/humble/setup.bash
source install/joint_bezier/setup.bash
python3 tools/prove_joint_bezier_scope.py log/joint_bezier/frozen_local \
  --output log/joint_bezier/scope_new
python3 tools/run_joint_bezier.py --root log/joint_bezier/frozen_local \
  --stage geometry --attempt new
python3 tools/analyze_joint_bezier.py log/joint_bezier/frozen_local \
  --stage geometry --attempt new
MPLCONFIGDIR="$PWD/log/joint_bezier/matplotlib" \
python3 tools/plot_joint_bezier.py log/joint_bezier/frozen_local --attempt new
```

新增 4 项 C++ 特征测试，覆盖符号闪烁、停车隔离、稀疏漏峰、非法工作量及证书预算耗尽。规划核心 7 个 CTest 目标通过，plan 的 flake8、lint_cmake、uncrustify 通过；Python 分析共 11 项通过。ROS 规划节点与 XML schema 在允许本机 DDS 的环境中复跑通过。原控制器节点测试读取用户保留的动态预算开启配置时有 1 项静态曲线用例失败；仅通过 `log/joint_bezier/static_tracker_test.sh` 为测试进程追加动态关闭覆盖后，22 项全通过。记录器及 Safety Gate 节点测试通过。

全包测试仍包含既有 DWA 格式／换行失败，不声称全包全绿。详见 `log/joint_bezier_test_results_latest.log`、`log/joint_bezier_static_tracker_tests.log` 和各次 CTest 日志。

Gazebo 驱动新增独立 `--dual-baseline --variants A` 入口，使用冻结路线和参数，强制动态预算关闭，并核对实际捕获的规划器／执行器开关及地图。该入口不沿用打开三功能的历史 `combined` 模式；此阶段只测试配置校验函数，未运行 Gazebo。

## 阶段 2：有界速度协调及双开离线验收

新增 `coordination_policy: bounded`，默认仍为 `legacy`。新策略从可达包络寻找瓶颈，尝试两端瓶颈对应的部分／完整峰值削减；按事件数、速度总变差、预计时间选优，保留纵向前后向传播和静态角加速度约束。物理连续弯组由曲率证书及实际限速区域识别，按短直线间距合并，停车点强制断开；每个弯组和全路径分别相对同几何未协调包络限制时间损失 2%。分析器独立重建弯组，不能靠程序报告一个过宽区间绕过局部门槛。

`process_frozen_speed` 只刷新速度帽，拒绝不完整证书；重复回放不会叠加前次降速。几何选定后的全局安全／长度／时间校核先于协调；协调失败恢复该几何的未协调速度帽，不丢弃已通过验证的几何路径。ROS 消息结构、PID/LQR、Safety Gate 及物理限制文件保持原样。

最终离线产物为 `log/joint_speed/bounded_20261102_bidirectional/`：原 8 场景加主路线反向，共 9 场景 × 5 次 × 6 组，270 组完整回放全部通过。A0/A/C 共用当前几何，B0/B/D 共用 dense 几何；每次均逐字段校对位置、切线、曲率、停车语义和连续证书，仅速度帽可变。反向几何通过反排位置、切线加 π、曲率反号和重排出段证书生成；对反向几何重新计算速度，没有反排前向协调结果冒充反向速度优化。

| 主路线，五次范围 | 当前双开 A | 优化双开 D |
|---|---:|---:|
| 正向预计通行时间 / s | 49.930–49.936 | 43.736–43.758 |
| 正向加减速事件 | 13 | 12 |
| 反向预计通行时间 / s | 49.930–49.936 | 43.543–43.660 |
| 反向加减速事件 | 13 | 11 |

全部局部弯组和全局 2% 预算、加减速可达性、停车及几何不变检查通过。第一次速度候选事件数不足，第二次仍未达到主路线目标，均完整保留。第三次修正了把微小速度隆起当作物理弯组的过细预算实现；2% 门槛不变，最终分组经独立分析器检查。图表：`speed_comparison_latest.png`、`speed_comparison_latest_reverse.png`，均为离线包络而非实际跟踪结果。

**基准预算口径校正。** 早期 `benchmark_speed_only` 经旧几何分支回放时，会按输入曲线重新计算时间门槛。现在六组显式沿用同一冻结安全输入的 `frozen_baseline_time × window_max_time_ratio`，与 `geometry_first` 原全局门槛一致。因此本节 A 的时间不能和阶段 0 的 44.67 s 跨预算计算收益；本节六组互相比较才具有相同预算。阶段 1 的独立几何收益仍使用同版本 A0/B0。

新增 7 项 C++ 速度测试及独立 Python 预算／反向证书测试，检查部分区间的平方速度积分、零损失回退、重复回放、停车隔离、可达性、非法证书以及保持几何的时间回退。最新规划核心与三项相关 lint 通过；Python 分析共 14 项通过。相关命令：

```bash
source /opt/ros/humble/setup.bash
source install/joint_bezier/setup.bash
cmake --build build/joint_bezier/plan -j2
cmake --install build/joint_bezier/plan
ctest --test-dir build/joint_bezier/plan --output-on-failure \
  -R '^test_(planning_grid|astar|path_processing|window_bezier|speed_coordination|joint_turns|bezier_refine_analysis|joint_bezier_analysis)$|flake8|lint_cmake|uncrustify'
python3 tools/run_joint_speed.py --root log/joint_bezier/frozen_local \
  --output log/joint_speed/new_run --include-reverse
python3 tools/analyze_joint_speed.py log/joint_speed/new_run
MPLCONFIGDIR="$PWD/log/joint_speed/matplotlib" \
python3 tools/plot_joint_speed.py log/joint_speed/new_run
```

## 阶段 3：冻结参考闭环验证入口

`tools/run_joint_gazebo.py` 在独立 Gazebo master / ROS 域中直接发布已验证的 `nav_msgs/Path` 和配对 `PathProfile`，避免现场重新规划污染 A/C 或 B/D 的几何对照。规划器保持启动以捕获配置，但此模式不发送新目标进行重规划。每组正反向各 3 次，控制组前提失败即停止候选实验。发布前核对实时膨胀地图，发布后核对记录的完整几何与速度帽、实际双开及动态关闭参数。

`tools/analyze_joint_gazebo.py` 分开报告全程与 TRACKING 状态，比较到达、意外停车、最大误差、弯道横向 P95、航向 P95 和速度跟踪误差。验收值位于 `src/plan/config/joint_tracking_validation.yaml`，启动时冻结到实验元数据。当前尚未宣称该闭环阶段通过。

### 2026-11-03 实跑：控制组终点前提未通过

首轮目录 `log/joint_gazebo/20261102_frozen_abcd/` 中，A 正向第一次到达（54.40 s）；第二次在记录校验阶段遇到空地图元数据，未保存完整运动结果，不能计为有效样本。原始轨迹及 `failure_record.json` 均保留。驱动补齐记录器订阅发现后的既定 2 s 稳定等待；缺失元数据、空轨迹或非有限误差现在输出失败原因，运动结果先于校验持久化。

修复后从头重跑于 `log/joint_gazebo/20261103_frozen_retry/`。A 正向第一次运行在既定 300 s 墙钟超时后仍为 TRACKING，ROS 记录时长 299.58 s，未到达。地图、规划器／控制器实际双开及动态关闭参数、冻结几何与速度帽全部核验通过；最大位置误差 0.030478 m，最大横向误差 0.030466 m，均低于 0.10 m。

终点诊断：沿路径剩余 0.019505 m，距终点欧氏距离 0.031694 m，大于既有 0.030 m 到达容差；参考速度与指令速度均为零，实速约 −2.5×10⁻⁷ m/s。此证据表明控制组再次出现终点停滞，不据此推断新几何或新协调的闭环效果。按计划停止 B/C/D，正反向各 3 次的联合闭环验收仍未完成；未改变控制器、到达容差或验收门槛。独立前提问题需先解决，再以新目录重跑整组，不能拼接两轮的成功样本。

分析器同时修正 TRACKING 的数值枚举筛选（记录中为 `2`），排除启动对准及无效几何样本；B/C/D 的横向、航向、速度 P95 均逐次与同方向同序号 A 比较，避免中位数掩盖单次退化。缺失或损坏的记录保留为无效结果，不中断整个分析。终点细节见 `A_outward_1/terminal_diagnosis.json`，完整结果见 `tracking_comparison.json`。失败运行中的 P95 包含长时间终点停滞，仅作为诊断，不能当作有效通行比较。

本轮新增 5 项记录／分析回归测试，Python 分析共 19 项通过。8 个相关算法／分析 CTest 目标通过，lint_cmake、uncrustify 通过；新增测试的两处 flake8 格式错误已修复，随后相关 Python CTest 与 flake8 复跑通过。日志为 `log/joint_bezier_20261103_final_tests.log`（保留首次 lint 失败）及 `log/joint_bezier_20261103_analysis_recheck.log`。补齐 plan 包的 numpy/pandas/yaml 测试依赖后重新构建成功，日志为 `log/joint_bezier_20261103_build.log`；XML schema 校验在允许访问官方 schema 的环境中通过，日志为 `log/joint_bezier_20261103_xml_recheck.log`。本轮没有改变 C++ 算法，既有离线几何／速度结果仍有效。最终分析器及跟踪分析依赖另存到该闭环目录的 `analysis_sources.tar.gz`，哈希见 `analysis_sources.json`。

```bash
python3 tools/run_joint_gazebo.py \
  --offline-dir log/joint_speed/bounded_20261102_bidirectional
python3 tools/analyze_joint_gazebo.py log/joint_gazebo/具体运行目录
# 本轮实际分析与回归命令：
colcon build --build-base build/joint_bezier --install-base install/joint_bezier \
  --packages-select plan --cmake-args -DBUILD_TESTING=ON
python3 tools/analyze_joint_gazebo.py log/joint_gazebo/20261103_frozen_retry
python3 -m pytest -q src/plan/test/test_joint_bezier_analysis.py \
  src/plan/test/test_bezier_refine_analysis.py
ctest --test-dir build/joint_bezier/plan \
  -R 'joint_bezier_analysis|flake8' --output-on-failure
ctest --test-dir build/joint_bezier/plan -R '^xmllint$' --output-on-failure
```

依照用户后续要求，本轮后续只修改工作区；暂存、提交和远程推送均由用户自行决定，已有暂存状态不改动。
