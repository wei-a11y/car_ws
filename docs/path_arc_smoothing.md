# 局部圆弧路径平滑

当前已进一步接入 [连续 LQR 跟踪与提前减速](continuous_lqr_turns.md)，包括辅助 profile、
更大半径代价选择及共享空间再分配。本文的初始圆弧设计说明以该文更新内容为准。

初始目标是改善 A* 理想规划路径的明显折角，保持已有节点和主路径接口：

`A* -> 去重/去共线 -> 可选 LOS shortcut -> 局部相切圆弧 -> 弧长重采样 -> yaw -> nav_msgs/Path`

算法仍在 ROS 无关的 `path_processing_core` 中；`astar_node` 只读取参数、发布两条
Path 并记录统计。`/plan/raw_path` 保留原始 A* 栅格中心序列，`/plan` 发布处理结果。
frame、stamp、首尾位置及失败/地图更新清空双路径的行为不变。

## 圆弧与保留拐点

- 每个非共线拐点尝试与前后直线相切的圆弧，左右转均支持。
- 转向角为 theta 时，切点距离拐点 `d = R * tan(abs(theta)/2)`。
  每条内部共享直线最多给相邻圆弧各使用一半长度，避免圆弧重叠；端部直线可以使用全长。
- 先尝试期望半径与空间允许半径中的较小值，再在它与最小半径之间均匀尝试有限个候选。
  每个候选单独检查，不能假定缩小半径就一定安全。
- 半径不足、碰撞、折返等无法圆滑的位置保留原折角，供停下再对齐的策略使用；
  不通过强行减小到最小半径以下来消除折角。
- 只保证局部候选搜索，不保证找到所有可能的安全曲线。贴障 shortcut、短段和连续紧弯
  可能保留多个拐点；没有承诺固定的停车数量。需要时可关闭 shortcut 比较结果。
- 每次成功规划日志输出 `rounded corners=N; retained corners=M`；统计针对 shortcut 后的拐点。

相切圆弧消除尖角并使理想几何切线连续，但直线/圆弧连接处曲率仍有跳变。
Path 是曲线的折线近似，保留圆弧采样顶点和所有回退拐点，不承诺曲率连续或速度/加速度连续。
yaw 仍指向下一个发布点，终点继承最后一段 yaw。

## 碰撞与离散误差

复用同一份已膨胀地图，不再次按机器人半径膨胀；unknown 和地图外策略不变。
圆弧角步长同时受弧长采样间距、最大角度和最大弦高误差限制。

每段小圆弧的弦高为 `s = 2*R*sin(delta/4)^2`。在地图局部坐标中，将障碍物闭合格子
边界按 s 外扩后，对弦做连续线段/矩形相交检查。整个小圆弧都处于这段弦的误差包络内，
因此该检查同时覆盖圆弧与弦，而非只检查离散点。误差包络碰到地图外也拒绝候选。
这种保守检查可能拒绝极近障碍但实际上安全的圆弧，误差参数不是额外机器人安全距离。

重采样仅在已验证的弦或直线段内插值，保留它们的顶点；最终再次对所有输出线段执行 LOS
检查。旋转地图 origin 先转换到地图局部坐标再验证。输出点数及每次候选采样均有资源上限。

## 参数

`src/plan/config/astar.yaml` 中 `post_processing` 下的启动参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `enable_arc_smoothing` | `true` | 关闭后恢复原折线路径处理 |
| `arc_radius` | `0.35` m | 期望半径，也是候选半径上限 |
| `arc_min_radius` | `0.20` m | 允许的最小圆弧半径，必须正且不大于期望半径 |
| `arc_radius_samples` | `6` | 每个拐点最多尝试的半径数量，范围 2–1000，含两端 |
| `arc_max_deviation` | `0.001` m | 允许的最大弦高误差；实际检查采用各弦的实际弦高 |
| `arc_max_angle` | `0.10` rad | 圆弧相邻采样点的最大转角，范围 `(0, pi/2]` |

这些是低速规划的初始调参值，不是测量得到的车辆物理能力。差速底盘能原地旋转，
这里的最小半径只约束前进圆弧，不约束停车转向。未来连续跟踪需同时满足 `v/R <= w_max`，
必要时在弯道降低速度。核心 C++ 配置默认关闭平滑以兼容既有调用，部署 YAML 显式开启。

## A* 正确性修正

累计代价修正为 `g(next) = g(current) + step_distance`，保留八邻域、octile 启发式和禁穿角规则。
当前目标为纯距离最短，不启用原注释中的靠障碍惩罚；测试中的 Dijkstra 同步使用相同距离代价。
测试独立累加返回路径的距离，并与报告代价及 Dijkstra 最优代价比较。
最优性仅针对原始栅格图，shortcut/圆弧后处理不声称连续空间全局最优。

## 验证与使用

在仓库根目录构建并测试；所有生成物留在仓库的忽略目录：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/arc_smoothing build --base-paths src \
  --build-base build/arc_smoothing --install-base install/arc_smoothing \
  --packages-select plan --cmake-args -DBUILD_TESTING=ON
source install/arc_smoothing/local_setup.bash
colcon --log-base log/arc_smoothing test \
  --build-base build/arc_smoothing --install-base install/arc_smoothing \
  --packages-select plan
colcon test-result --test-result-base build/arc_smoothing --verbose
```

若 `xmllint` 无法访问 `download.ros.org`，可从
[ROS 官方 REP 仓库](https://github.com/ros-infrastructure/rep/tree/master/xsd)
下载 `package_format3.xsd` 与 `package_common.xsd` 到 `log/arc_smoothing/`，
并创建本地 XML catalog，避免校验期间访问网络：

```bash
cat > log/arc_smoothing/schema_catalog.xml <<'XML'
<?xml version="1.0"?>
<catalog xmlns="urn:oasis:names:tc:entity:xmlns:xml:catalog">
  <system systemId="http://download.ros.org/schema/package_format3.xsd" uri="package_format3.xsd"/>
</catalog>
XML
export XML_CATALOG_FILES="$PWD/log/arc_smoothing/schema_catalog.xml"
```

本次验证使用此 catalog 运行测试。当前环境的 ament 会跳过 cppcheck 2.7，
应以 `colcon test-result` 汇总中的 skipped 为准，不能把它当作已执行的静态分析。

覆盖左右转/切点/半径/弦高、候选半径回退、短段/折返保留、相邻圆弧不重叠、
同一路径圆滑与保留拐点共存、弦安全但圆弧碰薄障碍、旋转地图、配置及资源边界。
随机地图对照独立线段碰撞检查，检查路径长度不增加及输出间距；ROS 测试检查开关确实改变
几何、原始路径/处理路径接口保持一致以及失败清空行为。

已有真实地图/定位及有效 TF 时，source 本次 install 后启动现有导航入口，
RViz 对比 `/plan/raw_path` 与 `/plan`，重新发目标，并查看 planner 日志中的两类拐点计数。
参数只读，修改后需要重启节点。无需启动 Gazebo 也可运行上述隔离合成测试；
它们不代表 Gazebo 实际运行验收或真实定位精度。

当前 LQR 已通过辅助 `PathProfile` 区分普通点和必要停车点，并使用连续弧长进度、
曲率前馈及提前减速。旧的逐段停车逻辑仅保留在显式关闭 profile 的兼容模式中，
部署配置默认开启 profile。更新接口、参数及测试见连续 LQR 文档。
