# 贝塞尔几何与速度协调优化

## 阶段 0：冻结基准与显式双功能入口

主路线为 `run_20261031_202141_283365` 的已膨胀地图和端点。进入贝塞尔前的安全路径由同版本程序重建，不称为历史原始中间结果。动态预算通过实验覆盖关闭，静态约束保留，工作区 `motion_limits.yaml` 未修改。

`bezier_benchmark` 新增兼容关闭的 `benchmark_use_config_switches`，显式使用配置中的功能开关；`benchmark_speed_only` 要求冻结 profile，并核对输出几何标识。原 benchmark 组合语义保持兼容。每段限速来源和加减速能力保存到 `_limits.csv`，其中包络端点速度与局部几何速度帽分别列出。

实验配置：`src/plan/config/joint_bezier.yaml`。驱动：`tools/run_joint_bezier.py`。分析：`tools/analyze_joint_bezier.py`。A0/A 使用同一份冻结几何；连续证书、停车语义、位置、切线、曲率逐字段核对。主验收区在候选实验前选择基准最长反向弯根窗口，其他根窗口保护不退化；米数不进入生产策略。

产物：`log/joint_bezier/frozen_local/`，包含输入哈希、源码与二进制快照、逐候选诊断、最终节点、重建控制点、解析密集采样和连续证书。日志被 git 忽略，分享报告时需同时保存。最初沙箱 DDS 初始化失败位于 `log/joint_bezier/frozen/`，不计入有效样本。

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
