# Phase 2：2D Planning Grid

## 运行接口

`plan/planning_grid_node` 输入 `nav_msgs/msg/OccupancyGrid`，保留当前内部快照，
输出两个同类型的调试地图：

| 角色 | 默认 topic | 内容 |
| --- | --- | --- |
| 输入 | `/map` | 原始概率数据及实际 frame/origin |
| raw grid | `/plan/raw_grid` | 输入数据完整副本，保留原概率和 unknown |
| inflated grid | `/plan/inflated_grid` | `100` 不可通行、`0` 可通行、`-1` 可通行但未知 |

以上全部可配置或 remap。输入和输出 QoS 均为 reliable、transient_local、depth 1；
输入地图发布方也必须提供该 QoS。wrapper 不查询机器人 pose，也不新增 localization。
`expected_frame` 为空时接受实际输入 frame；非空时严格匹配。
输出完整保留输入 header、stamp、resolution、width/height、origin 和 map_load_time。
此处的 `grid_debug` 仅用于合成地图验证，不代表项目已经确定了 map frame。

## 核心与膨胀语义

`PlanningGrid` 核心库只使用 C++ 标准库，独立于 ROS。
内部同时保留原始 occupancy、阈值后的障碍 mask 和 inflated grid。
索引为 `y * width + x`，`cell_to_world` 返回 cell 中心，
`world_to_cell` 使用 origin 平移和逆 yaw 旋转后向下取整，拒绝越界或非有限坐标。
单元测试覆盖带平移及 90 度 yaw 的地图。

- 概率值 `>= occupied_threshold` 为障碍；阈值允许 `[1,100]`。
- `unknown_is_obstacle: true` 时 unknown 是膨胀种子；设为 false 时 unknown 可通行，
  未被其他障碍膨胀覆盖的 unknown 在 inflated debug grid 中仍保持 `-1`。
- 有效机器人半径 `R = robot_radius + safety_margin`，不重复配置独立 inflation_radius。
- 两遍精确欧氏距离变换，时间/空间复杂度均为 `O(width * height)`。
  障碍 cell 是方块，因此对其使用半对角线界限：
  距离种子中心 `<= R + resolution * sqrt(2) / 2` 的 cell 中心设为不可通行。
  这比将障碍当作无面积的点更保守。
- `inflate_map_boundary: true` 将地图外视为不可通行：距离边缘 `<= R` 的中心被阻塞。
- 此 grid 的通行判定针对机器人中心位于 cell 中心；不会证明跨 cell 的连续曲线安全。

## 已核验机器人尺寸

尺寸来源是当前 `src/urdf/urdf/car_urdf.urdf` 的全部 collision 几何，
结合实际 STL 顶点、mesh scale、joint origin 和 collision origin 转换到 `base_footprint`。
轮子圆柱使用包围球界限，覆盖任意轮子转角。

- XY 包络：`x=[-0.258004427, 0.243000001] m`，
  `y=[-0.242902264, 0.242909551] m`。
- 基于最大绝对 x/y 的保守外接圆：`0.354360458 m`。
- YAML 的 `robot_radius: 0.355` 是向上取整到毫米后的已核验尺寸，包含外壳/突出部件。
- `safety_margin: 0.05` 是本阶段的可调安全余量，不是模型尺寸。
- wheel radius/wheel separation 是驱动参数，不能作为整车 robot_radius。

复算工具只依赖 Python 标准库，同时检查 YAML 半径不小于实际包络。
URDF/mesh 后续变化时必须重新执行；不支持的几何会报错，不能静默猜测。

```bash
python3 src/plan/tools/verify_robot_radius.py
```

## 参数与无效数据

全部参数位于 `src/plan/config/planning_grid.yaml`。除标准 `use_sim_time` 外，
接口/膨胀参数在启动时读取且只读，变更后重启，避免部分更新造成快照不一致。
`max_cells` 限制地图 cell 总数；维度、长度、resolution、origin 和 occupancy 范围均验证。
ROS wrapper 额外检查非空 frame、单位 quaternion 和 yaw-only origin。

无效输入会被拒绝，保留最后有效快照及原始 stamp，不发布假冒的新数据。
下游仍必须检查时间/地图有效性，不能把 retained 调试地图视为新的导航授权。
当前没有 A*、LQR、dynamic obstacle 或速度命令输出。

## 构建与测试

从仓库根目录运行，生成文件保留在仓库内：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/phase2_grid build --base-paths src \
  --build-base build/phase2_grid --install-base install/phase2_grid \
  --packages-select plan
source install/phase2_grid/local_setup.bash
mkdir -p log/phase2_grid/ros
export ROS_LOG_DIR="$PWD/log/phase2_grid/ros"
colcon --log-base log/phase2_grid test \
  --build-base build/phase2_grid --install-base install/phase2_grid \
  --packages-select plan
colcon test-result --test-result-base build/phase2_grid --verbose
```

gtest 验证 grid/inflation/坐标转换，并用暴力欧氏距离作为随机地图的独立对照。
ROS 测试在独立 localhost domain 下验证真实 publisher/subscriber、元数据保持、
无效输入拒绝、late-joiner QoS、参数错误及 remap 自反馈保护。

## 启动与 RViz 验证

```bash
ros2 launch plan planning_grid.launch.py
# 可用 params_file:=<自己的 YAML> 覆盖参数。
```

没有地图 publisher 时节点等待 `/map`，无需启动定位或加载仓库中未选定的地图。
下面的合成图只用于验证，最好在单独终端组统一设置 `ROS_DOMAIN_ID`、
`ROS_LOCALHOST_ONLY=1` 并启动 grid 节点。

```bash
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable \
  /map nav_msgs/msg/OccupancyGrid \
  '{header: {frame_id: grid_debug}, info: {resolution: 0.5, width: 7, height: 7, origin: {position: {x: 2.0, y: -1.0}, orientation: {z: 0.7071067811865476, w: 0.7071067811865476}}}, data: [0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,100,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,-1,0, 0,0,0,0,0,0,0]}'

ros2 topic echo /plan/raw_grid --once --qos-durability transient_local
ros2 topic echo /plan/inflated_grid --once --qos-durability transient_local
ros2 topic info /plan/inflated_grid --verbose
rviz2 -d src/plan/config/planning_grid.rviz
```

RViz 的 Fixed Frame 设置为实际地图 `header.frame_id`（上例 `grid_debug`），
两个 Map display 采用 reliable/transient_local。
分别切换 Raw Grid / Inflated Grid：raw 保留中心障碍和 unknown；inflated 显示扩大禁行区。
带 yaw 的图应按 origin 正确平移/旋转，无须伪造 TF。
