# 曲率连续路径、纵向 PID 和全程跟踪评估

## 本次行为

保持 `Goal -> A* -> Path Processing -> controller -> Safety Gate -> /cmd_vel`。
`/plan` 仍为 `nav_msgs/Path`，A* 提供几何路线，速度参考由控制器生成。
本次没有 Git 暂存、本地提交或推送。

- 路径处理启用对称回旋线入口、可选的定曲率圆弧、回旋线出口。
  曲率沿路程线性变化；小转角可以只有两段回旋线。
  重新计算入口/出口位置，不在原圆弧上仅修改曲率数字。
- 搜索允许半径、过渡长度，并重新检查整段曲线及离散弦线。
  检查包络包含曲线到弦的偏差和数值积分误差。
  无法满足空间、曲率变化率或碰撞约束时，保留停车转向点。
  相邻转弯的实际裁剪长度参与空间分配。
- 现有辅助 `PathProfile` 新增 `curvature_continuous`，明确节点曲率插值语义。
  `false` 保留旧圆弧的出段曲率语义；`true` 在普通节点间连续插值。
  普通采样点不停车，必要停车点不跨越插值或预瞄。
  **消息结构已改变，需要重新编译 common_msg、plan、controller 并重启相关节点。**
- 正常 profile 模式中，纵向速度使用 PID，横向及航向使用 LQR。
  PID 对里程计速度闭环，包含参考前馈、测量微分滤波、积分限幅和条件积分抗饱和。
  停车、对准、新路径、数据过期、时钟异常和看门狗停车都会复位 PID。
  最终速度受曲率、制动距离、速度和加速度约束，积分不能突破弯道/停车限速。
  旧 XY-only 保守兼容模式保持旧行为；部署继续使用 require_path_profile: true。
- LQR 按最终协调速度查询预计算的增益表。
  转弯前馈仍为 `omega_ff = v_cmd * kappa_ref`，并保留反馈角速度余量。
- 速度规划同时限制角速度、横向加速度、曲率变化率。
  角加速度前馈预算分别留给 `a*kappa` 和 `v^2*kappa'`。
  反向制动包络处理未来弯道和必要停车，运行时参考和指令限速处理加速。
  速度指令必须满足当前制动包络；必要时该约束优先于指令减速度平滑。

本次保留巡航速度 0.15 m/s，是为了对比实验。算法没有写死该速度。
之后可以提高 v_nominal/v_max，但需同时核对角速度、加减速度、控制周期、
安全门和底盘限制，并重新评估误差。当前底盘配置的线速度上限为 0.5 m/s。
合成模型中 0.35 m/s 的验证不等于 Gazebo 或实车在该速度下已经验证。

## 调试参数

参数在启动时读取；修改 YAML 后重启对应节点。源码中保留算法实现，YAML 存放调试值。

| 文件 | 经常调整的参数 |
| --- | --- |
| `src/plan/config/astar.yaml` | `post_processing.arc_radius/arc_min_radius/arc_radius_max`、`transition_length/min_length`、`max_curvature_rate`、采样精度、半径与保留折角代价 |
| `src/controller/config/lqr_tracker.yaml` | `v_nominal/v_max/w_max`、线加减速度、角加速度、横向加速度、前馈预算、`speed_pid.*`、LQR Q/R、增益表范围/点数、预瞄距离 |
| `src/controller/config/path_tracking.yaml` | 记录话题、参考坐标系、输出路径、误差阈值、弯道判断阈值、停车判据、图像大小和分辨率 |

`speed_pid.kd` 初始为 0，接口支持完整 PID。先调 P/I，确认速度反馈噪声后再增加 D。
纵向指沿车体前进方向的速度；差速底盘没有独立的侧向速度执行器。
`lookahead_distance` 仍是诊断预览，转向前馈使用 `feedforward_preview_distance`。

## 构建与启动

在仓库根目录执行；已有正常工作区 underlay 时可使用独立验证 overlay：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --build-base build/smooth_pid --install-base install/smooth_pid \
  --packages-select common_msg plan controller --cmake-args -DBUILD_TESTING=ON
source install/smooth_pid/setup.bash
```

各新终端使用同样的 source 顺序，最后 source 本次 overlay。
先启动记录器，再启动导航、发送目标：

```bash
ros2 launch controller path_tracking.launch.py \
  params_file:="$PWD/src/controller/config/path_tracking.yaml"
```

```bash
ros2 launch car_description navigation.launch.py controller_type:=lqr \
  planner_params_file:="$PWD/src/plan/config/astar.yaml" \
  lqr_params_file:="$PWD/src/controller/config/lqr_tracker.yaml"
```

也可直接运行源码记录器，显式传入参数：

```bash
python3 src/controller/scripts/record_path_tracking.py --ros-args \
  --params-file src/controller/config/path_tracking.yaml
```

结束后对最近一次记录进行分析（默认不弹窗）：

```bash
python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml
```

或指定某个 run / path 目录，用 `--show` 弹窗：

```bash
python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml \
  --data-dir data/path_record/run_具体时间/path_0001 --show
```

## 图像和指标

每次启动单独建立 `run_时间/`，每次接收非空路径单独建立 `path_编号/`。
不覆盖其他实验，不把重新规划前后的实际轨迹与同一条参考路线混算。
空路径停止记录；目标到达或跟踪故障后停止写该 epoch 的重复样本。
记录器应在目标发送之前启动。重复使用时间戳的路径拒绝记录，避免旧样本错配；
正常 planner 用规划时间戳标识路径。

`tracking_overview.png` 包含且仅包含三类图：

1. 整条处理后参考路径与实际 TF 轨迹的等比例叠加图，颜色表示横向误差，标出必要停车点和终点。
2. 带正负号的横向误差折线图。
3. 带正负号的航向误差折线图。

误差横轴默认路径进度；把 YAML 中 `analysis.error_axis` 改为 `time` 可以查看停顿期间的变化。
曲线区域以背景色标出。误差直接使用控制器当前周期的局部进度投影结果，
不会在全路径任意寻找最近线段，避免交叉/回头路误匹配。

`tracking_summary.json` 包含全程及直线/弯道分组误差、RMSE、P95、最大误差、
超阈值比例、路径覆盖率、是否到达终点、异常停车次数和持续时间。
未跑完的路线会明确显示覆盖率和 goal_reached=false。
异常停车定义为 TRACKING 状态下、远离起点和必要停车点、实际速度持续低于阈值。
它是用于调试的统计判据，不能识别所有安全门停车或定位故障原因。

`actual_path.csv` 另存参考/指令/反馈速度、角速度、曲率、前馈、状态和仿真时间，
便于后续排查，但默认不再增加其他类型的图。
`events.csv` 保存规划/状态变化。每个 epoch 独立评估，run_summary.json 汇总各 epoch。

新增 `/controller/lqr/tracking` 使用 Float64MultiArray，维度标签列明字段名，
包含同一控制周期的位姿、误差、速度、进度、ROS 时间和关联 Path 时间戳。
原 `/controller/lqr/debug` 的 11 项布局保持不变。
TF 轨迹是定位估计，不代表 Gazebo 真值；本次不新增真值接口。
旧记录缺少同步误差和路径版本，分析器会提示重新记录。

## 验证命令

```bash
source /opt/ros/humble/setup.bash
source install/smooth_pid/setup.bash
colcon test --build-base build/smooth_pid --install-base install/smooth_pid \
  --packages-select plan controller --ctest-args \
  -R 'test_path_processing|test_continuous_tracking|test_lqr_tracker$|test_astar_node|test_lqr_tracker_node|test_path_recorder|test_tracking_analysis' \
  --output-on-failure
colcon test-result --test-result-base build/smooth_pid --verbose
```

独立 build/install 目录在 test 时也必须配对，避免加载默认 install/ 下的旧消息。
本次只运行与修改相关的测试，不把既有其他模块的未运行测试标为通过。

## 本次验证结果

- common_msg、plan、controller 在独立 overlay 中构建成功。
- 相关用例共 100 项通过：路径处理 26、planner ROS 24、连续跟踪/PID 8、
  旧 LQR 核心 17、LQR ROS 22、记录器 1、分析指标 2。
  `colcon test-result` 汇总含测试包装记录共 103 条，0 errors、0 failures、0 skipped。
- 修改的 C++ 文件通过 ament_uncrustify；Python 文件通过 ament_flake8；git diff --check 通过。
- 验证了记录器 launch 参数解析、启动和 SIGINT 正常退出。
- 三弯道合成模型以 0.35 m/s 为巡航目标完成跟踪，未发生异常停车；
  横向 RMSE 约 0.0027 m，P95 约 0.0067 m，最大值 0.0100 m（含初始 0.01 m 偏差）。
  图像已实际生成并检查：`log/smooth_pid/synthetic_validation/path_0001/tracking_overview.png`。
  **这些是合成模型结果，不是 Gazebo 实跑或定位真值的测量。**
- 未启动 Gazebo 实跑；需要在固定地图、初始位姿和目标下采集新数据，对比三图及统计指标。
