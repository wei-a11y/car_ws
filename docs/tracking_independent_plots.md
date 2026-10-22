# 独立高清图表与运行数据记录

本轮仅扩展记录和离线分析，不改变规划、PID/LQR、Safety Gate 或运动参数。不考虑侧滑，不新增横向速度或 Gazebo 真值接口。
各单项图默认保存 300 DPI PNG（3600×1500 像素），横坐标仅为路径进度。标题、坐标轴、图例、色标和事件标注为中文；文件名及 CSV/JSON 字段名保持稳定。
新增一张全程纵向速度与位置距离误差双纵轴合并图，同时保存 PNG 和 SVG。SVG 中的曲线与中文字形为矢量，可在浏览器中放大保持清晰。
PNG 是位图，不保证无限放大清晰；需要更高像素时在 YAML 中提高 `dpi` 或 `figure_size`。

## 构建与记录

本次无需修改或重建消息结构。已有 joint_turn overlay 的工作区可执行：

```bash
source /opt/ros/humble/setup.bash
source install/joint_turn/setup.bash
colcon --log-base log/tracking_figures build \
  --build-base build/tracking_figures --install-base install/tracking_figures \
  --packages-select controller --cmake-args -DBUILD_TESTING=ON
source install/tracking_figures/setup.bash
ros2 launch controller path_tracking.launch.py \
  params_file:="$PWD/src/controller/config/path_tracking.yaml"
```

导航继续使用自己的现有启动参数；先启动记录器，再发送目标。源码记录器也可直接运行：

```bash
python3 src/controller/scripts/record_path_tracking.py --ros-args \
  --params-file src/controller/config/path_tracking.yaml
```

记录仍按 `data/path_record/run_时间/path_编号/` 区分，不混合不同路径。每个控制器有效采样独立写入 CSV，使用仿真时间；这不是对连续物理时间的无限采样。
空路径停止当前记录；终点或跟踪故障的最后一个样本保存一次后停止重复写入。Path/Profile 仍按完整路径和时间戳配对。

## 单次分析与输出

```bash
python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml \
  --data-dir data/path_record/run_具体时间/path_0001 --no-show
```

`--data-dir` 支持一个 path、一个 run 或记录根目录。省略时继续采用原来的最新 run 选择规则；最新 run 为空时不会悄悄换成旧实验。
`--show` 默认开启，逐张显示；批量导出建议用 `--no-show`，再单独打开所需的 PNG。关闭当前窗口后才会继续生成下一张；`--no-show` 会连续生成全部文件。

每个 path 的 `figures/` 包含：

| 文件前缀 | 内容 |
| --- | --- |
| `trajectory_overlay` | 等比例参考/TF 路径，横向误差色标、起点、终点、必要停车点 |
| `longitudinal_speed` | `v_ref` 目标、`v_cmd` 控制指令、`v_actual` 里程计前向速度 |
| `angular_velocity` | `w_ff` 前馈、`w_cmd` 控制指令、`w_actual` 里程计角速度 |
| `curvature` | 控制器用于前馈的参考曲率，可能位于当前位置前方预瞄点 |
| `acceleration` | 按实际时间差分的纵向加速度估计及低通结果，不是独立加速度传感器 |
| `lateral_error` | 带符号横向误差，米 |
| `heading_error` | 航向误差，图中为度，CSV/JSON 保持弧度 |
| `position_error` | 当前位置到本周期控制器选定投影点的二维距离，米 |

除地图 XY 轨迹叠加图外，所有单项曲线仅生成全程 `_progress` 版本，每个物理量保留一张单项图。
`plot_tracking_errors: false` 关闭额外的“正常跟踪阶段”误差图，重新导出会清理旧的 `_tracking_progress` 图；分阶段统计仍然保留。
不再输出时间轴图或逐弯局部图，所有转弯区间在全程曲线上统一标注。重复进度点不去重；无效数据和长时间缺口处断线。
路径进度轴不表示停车持续时间；时间戳与持续时间仍完整保存在 CSV/JSON 中。

`speed_position_error_progress.png` / `.svg` 是同一张全程合并图：左轴为参考、指令及实际纵向速度（m/s），右轴为位置距离误差（m），附位置距离误差阈值。
横轴覆盖规划路径从 0 到终点的完整长度；未运行到的部分保持空白，不补造数据。各转弯区间使用橙色背景。原有速度与位置误差单项进度图也保留。
用浏览器打开 SVG，可用浏览器缩放查看曲线和文字；PNG 为快速预览。
灰色背景标注对准、红色背景标注制动、橙色背景标注转弯；绿/红三角标注持续加/减速区间起点；竖线标记最大误差、重对准、停车及到达等事件。
`plots_manifest.json` 列出本次导出的文件、格式、中文标注语言和跳过原因。
重新导出后清理该目录中已知的旧时间轴图、逐弯图和不再输出的图表格式，任意用户图片及原始记录不删除。
合并图使用独立的 `combined_formats`，默认保留 PNG 与 SVG；其他单项图默认只保留 PNG。旧 `tracking_overview.png` 不再生成，但不会删除历史文件。

## 数据与指标含义

- `actual_path.csv` 保持旧字段并追加 `nearest_x`、`nearest_y`、`position_error`。距离严格来自同一控制器消息中的 TF 位置和局部投影，不重新全局找最近点。
- `tracking_error.csv` 是派生分析数据，包含原始行、位置距离、原始/滤波加速度、控制状态、几何类型、纵向动作、转弯编号；不覆盖原始 CSV。
- `tracking_summary.json` 保留旧指标，新增位置距离、TRACKING/控制状态/几何/纵向动作分组、每弯指标与事件。
- MAE 是平均绝对误差，RMSE、最大绝对误差、P95、`exceed_percent` 采用有效样本口径。新增 `exceed_duration_sec`、`exceed_time_percent` 使用有效左端样本保持的采样区间，排除无效/缺失测量及大于 `max_sample_gap` 的时间缺口；没有可统计时长时比例为 null。
- `unexpected_stop_count` 保持旧含义：TRACKING 下远离起点和必要停车点的持续低速。它不代表所有 BRAKING/ALIGNING；实际初始停车、计划停车、其他停车和重对准还要查看 `events`。
- 每个事件保存时间、ROS 时间、进度、XY、控制状态、速度和误差；加减速/停车事件还保存结束时间和时长。
- 最大偏差是相对几何路径的误差，不是实际与规划 CSV 同行相减，也不是对具有时间戳的目标轨迹求跟踪延迟。TF 位置是定位估计，不是 Gazebo 真值。

旧 CSV 缺投影坐标时，位置距离统计为 null、相关图跳过，已有横向/航向误差和速度图正常输出。不会用当前配置、横向误差或全局最近点重建历史位置距离。

## 参数与实验快照

所有图表和分析调试值在 `src/controller/config/path_tracking.yaml`。默认每图 12×5 英寸、字号 12、线宽 1.5；默认 `formats: [png]`；`font_families` 配置中文字体优先级，当前环境使用 Noto Sans CJK JP（含中文字形）。
格式、分辨率 `dpi`、图表清单、TRACKING 视图、合并图开关 `plot_speed_position`、合并图格式 `combined_formats`、图例列数 `combined_legend_columns` 和阈值均可配置。
`plot_axes` 仅允许 `[progress]`，旧的逐弯图配置已移除；单项图使用 `formats: [png]`，合并图使用 `combined_formats: [png, svg]`。
加速度低通时间常数 0.15 s，超过 0.20 s 缺口重置；加减速阈值 0.02 m/s²、持续 0.20 s。转弯阈值 0.10 m⁻¹，相邻曲线间隔同时不超过 0.30 m 和 0.50 s 才合并；无效数据、离开 TRACKING 或经过必要停车点时断开。
这些标签不修改车辆运动。差分尖峰不等于真实冲击，滤波曲线也不能代替原始速度。

每条路径的 `experiment.json` 异步记录 `/planner`、`/lqr_controller`、`/planning_grid`、`/safety_gate`、`/diff_drive_controller` 的实际参数，以及膨胀地图元数据/摘要、最终路径端点及最近收到目标。
节点名、地图/目标话题、采集开关和超时均可配置。标准参数服务读取不阻塞采样；未启动、超时、错误和提前关闭均有明确状态。
快照记录采集时间，明确 `atomic_snapshot: false`；最近收到目标只是观察到的消息，不代表带请求 ID 的强因果关联。历史记录没有快照时不会从当前 YAML 补造。

## 显式两次实验对比

```bash
python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml \
  --baseline data/path_record/run_基线时间/path_0001 \
  --candidate data/path_record/run_新时间/path_0001 --no-show
```

两个参数必须同时提供且各自指向单个 path，不与 `--data-dir` 混用。默认保存到 candidate 的 run 下 `comparisons/基线run_路径_vs_候选路径/`，可通过 `--output-dir` 指定其他输出目录。
每个物理量在独立图中比较，速度用实际里程计值；同图共享坐标范围，横轴保留各自路径进度，取消时间轴对比图。
`comparison_summary.json` 保存两边统计、candidate 减 baseline 的指标差、地图/端点/路线/参数差异、缺失快照和跳过说明。
不会自动匹配实验、反转回程或平移旋转路径；没有共同记录坐标系时不画轨迹叠加。即使画在同一进度轴，也不代表同一个物理位置，路线不同不能直接得出局部控制改善结论。

## 验证命令

```bash
source /opt/ros/humble/setup.bash
source install/tracking_figures/setup.bash
colcon --log-base log/tracking_figures_test test \
  --build-base build/tracking_figures --install-base install/tracking_figures \
  --packages-select controller \
  --ctest-args -R 'test_tracking_analysis|test_tracking_plots|test_path_recorder' \
  --output-on-failure
colcon test-result --test-result-base build/tracking_figures --verbose
```

导出测试检查单项图仅使用路径进度、全程双纵轴合并图、中文标注、PNG 为 3600×1500、SVG 无嵌入位图、数据缺口断线和重复进度停车点保留；还覆盖时间轴/逐弯旧图清理及任意用户文件保留。
真实记录复核使用 2026-10-21 已有往返 CSV，属于离线重绘与统计，不是新的 Gazebo 实跑。历史数据没有投影坐标，所以真实记录的二维位置距离图不可用；位置距离计算与导出由包含真实消息投影字段的记录测试及明确标注的合成样例验证。

## 首次实现验收记录（2026-10-21，含格式调整前的历史结果）

- controller 在 `build/tracking_figures` / `install/tracking_figures` 构建成功，最终脚本与 YAML 已安装。
- 分析 14 项、图表/对比 5 项、ROS 记录/参数采集 2 项，共 21 项用例通过；colcon 包含包装记录汇总为 24 条，0 errors、0 failures、0 skipped。
- 修改的 8 个 Python 文件通过 `ament_flake8`，`git diff --check` 通过。导航算法、ROS 消息及已有用户运动参数没有改动。
- ROS 记录测试覆盖投影距离 0.5 m、Path/Profile 配对、跨路径旧样本拒绝、终点样本仅记录一次，以及实际参数服务成功/超时/提前退出。参数中的非有限默认限制使用带类型标记的 JSON 对象保存，不写入非法 NaN/Infinity 数字。
- 矢量验收检查 SVG 没有 `<image>`，PDF 没有图像对象，默认 PNG 为 3600×1500；人工检查了完整速度图、TRACKING 航向误差图和转弯局部速度图。

已有真实往返记录的离线示例：

- 去程：`data/path_record/run_20261021_192905_828717/path_0005/figures/`
- 回程：`data/path_record/run_20261021_192905_828717/path_0006/figures/`
- 显式对比：`data/path_record/run_20261021_192905_828717/comparisons/run_20261021_192905_828717_path_0005_vs_path_0006/`

回程 `tracking_summary.json` 的 events 同时识别出约 52.3 秒处的 planned_stop 与 realignment；旧指标 unexpected_stop_count=0 不能解释为“回程没有停车”。
两条路线方向及端点不同，对比产物明确保留这些差异，不得据此直接认定同路段性能改善。

位置距离图的独立合成样例位于 `log/tracking_figures/synthetic_validation/synthetic_path_0001/`。
它是测试输入，不是 Gazebo 或运动学/动力学仿真结果，不用于报告运行性能。本轮没有新增 Gazebo 实跑。

本轮未执行 Git 暂存、本地提交或远程推送。`data/`、`log/` 按原有忽略规则保留为本地验证产物。

## 中文 PNG 输出调整

当前实际记录示例：`data/path_record/run_20261021_205448_297127/path_0001/figures/`。
该记录包含投影坐标，能够生成位置距离误差图。重新分析使用上述 `--no-show` 命令；
图内文字改为中文，数值单位保持标准符号，路径编号保留原标识。本次调整不改变统计或车辆运动。

本次格式/中文调整验证：19 项分析与绘图测试通过，4 个 Python 文件通过 `ament_flake8`，
实际记录导出 37 张 3600×1500 PNG，绘图目录没有 SVG/PDF 残留；已人工查看中文速度图。
复现本次相关测试：

```bash
MPLCONFIGDIR=/tmp/tracking_plots_mpl python3 -m pytest -q \
  src/controller/test/test_tracking_plots.py \
  src/controller/test/test_tracking_analysis.py
ament_flake8 src/controller/scripts/tracking_plots.py \
  src/controller/scripts/tracking_analysis.py \
  src/controller/scripts/analyze_path_tracking.py \
  src/controller/test/test_tracking_plots.py
```

构建继续使用文首的 controller 构建命令（日志目录本次为 `log/tracking_figures_png`）。

## 全程进度合并图调整（2026-10-22）

离线重绘当前已有真实记录：

```bash
python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml \
  --data-dir data/path_record/run_20261021_205448_297127/path_0001 --no-show
```

新增合并图位于该记录的 `figures/speed_position_error_progress.svg`，PNG 同名。
旧 CSV 缺少投影点时不生成合并图，在 `plots_manifest.json` 中记录原因；单项速度图仍然生成。
统计、转弯识别与加速度估计继续使用原始时间，只有图表横坐标改为路径进度，不改变规划和控制算法。
构建仍使用文首的 controller 构建命令；相关测试使用上节的 pytest / ament_flake8 命令。

本次验收：controller 构建成功（日志 `log/tracking_progress_plots`），21 项分析/绘图测试通过，
5 个 Python 文件通过 `ament_flake8`。已有真实记录重绘得到 12 张 PNG 与 1 张合并 SVG；
输出目录无时间轴图和逐弯图。SVG 无内嵌位图，已人工检查合并图的中文双纵轴与图例。
这属于已有 CSV 的离线重绘，没有新增 Gazebo 实跑；未执行 Git 暂存、提交或推送。
