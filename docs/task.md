# task 避障、移障与复原：实施计划与进度

更新时间：2026-10-10。当前分支：`task`。

## 授权范围与目标

- 功能实现、配置、测试、场景副本及构建/运行产物仅在 `src/task`。
- 用户后续明确要求新增本文件，因此 `docs/task.md` 是范围外唯一获授权的进度文档；其他目录仍只读。
- 保留用户已暂存的 task 点位、URDF 和控制器配置，不覆盖、不回退。
- 在现有取货→送货→回 home 流程中加入路段进入检查、绕行/移障统一评分、抓取验证和物体复原。
- 本阶段使用现有 Gazebo 底盘和 LiDAR，感知及抓取是模拟后端；不声称实现真实 YOLO、关节轨迹或接触抓取。

## 整体实施计划

1. **规划与几何核心**：通过轻量 Python 绑定复用 plan 的膨胀、A* 和路径处理；检查观测走廊，保留折点，评估绕行、持物复原、临放复原的完整代价与可行性。
2. **Gazebo 后端**：通过指定参考实体及同时间 TF 转换坐标；只识别可见的登记物体；执行虚拟预抓取、张开、接近、闭合、竖直抬升、搬运、放置和多帧验证。
3. **任务调度与恢复**：候选 Path 先经过 task，检查后逐段交给 LQR；分离任务目标和临时子目标；所有速度继续经过 Safety Gate；持久保存未完成的复原事项。
4. **启动与场景**：在 task 内生成 world 副本并加载已安装的物体状态插件；统一加载原参数链和 task 覆盖；提供登记障碍的场景工具。
5. **测试与交付**：单元测试、ROS 节点/导航联动、Gazebo 场景验证；记录实际结果与阻塞项，编写简要功能/启动说明。

## 当前模块状态

| 模块 | 当前状态 | 已完成工作与剩余检查 |
| --- | --- | --- |
| 环境与改动保护 | 已完成 | 确认 `task` 分支，记录用户原有四个文件的内容哈希，确认仅根 AGENTS.md 适用 |
| 启动/world 工具 | 实现、安装与单元验证完成 | 5 项测试通过；新启动文件、受限 world 副本、依赖声明和 Gazebo 缓存路径配置完成 |
| 规划评估核心 | 实现与单元验证完成 | 11 项测试通过；复用真实 C++ A*，覆盖路径分段、遮挡/无效扫描、静态占据保护、匿名激光障碍、无路淘汰、方案代价切换、临放双侧可达和过期观测 |
| Gazebo 模拟后端 | 实现与单元验证完成 | 11 项服务模拟测试通过；覆盖非单位坐标变换、完整抬升/复原、假成功拒绝、失败重试、夹持取消、服务/TF 失效、复原误差和新观测要求；真实 Gazebo 验收受阻 |
| 任务调度 | 实现与自动化验证完成 | 8 项调度保护检查、6 项完整导航/故障联动通过；临时点栅格量化和停车容差边界已修正 |
| 回归与 Gazebo 验收 | task 自动化回归完成；Gazebo 待范围授权和现场验收 | task 共 64 个 pytest 用例、8 个 CTest 组全部通过；plan/controller 各一个既有失败见下文。联动采用真实 A*/LQR/Safety Gate 加理想底盘与兼容状态服务，不能替代 Gazebo 物理场景验收 |
| 使用说明 | 已完成 | `src/task/OBSTACLE_RECOVERY.md` 已包含功能、模拟边界、配置、启动步骤、实际测试结果及未完成验收事项 |

## 已确定的行为

- 无障碍先检查后放行；未知/遮挡空间不能按静态空白地图直接通过。
- 默认对绕行与移障统一计算归一化时间/估算做功代价；无法复原的移障方案直接淘汰。
- 袋子持物通过后原位放回；box 示例临时放置后重抓复原；凳子只绕行。
- 抓取采用预抓取点张开→接近→闭合→地图 +Z 抬升；服务响应成功不等于抓取验证成功。
- 抓取最多三次；最终无可行方案则停稳并结束本轮，下次 start 从取货开始。
- 未完成的复原事项跨进程保存，不能用新 start 擦除。
- 不修改 Safety Gate 阈值、原导航源码、原 world 或 URDF。

## 验证记录

- 本次已执行 Python AST 语法检查，通过。
- 启动/world 工具 5 项测试通过（模块实现阶段执行）。
- plan、controller、car_description、task 构建通过（47.3 秒）；car_description 有未使用 PIC 变量的 CMake 提示，不影响构建结果。
- 新增基础测试：23 项通过（规划核心 8、Gazebo 后端 9、启动/world 5、复原账本 1）。
- 原 task ROS 节点回归：10 项通过（7.00 秒）。
- 恢复任务完整联动：4 项通过，165.96 秒。覆盖无障碍、袋子、临放箱子、凳子，每轮验证最终 TASK_DONE、取送事件不重复，以及需要时的物体复原和账本清除。
- 故障联动补充：2 项通过，51.96 秒（抓取失败两次后成功、扫描丢失后停稳失败）；旧到达反馈、重复反馈、解除屏障和未完成复原账本等 5 项检查通过，0.30 秒。
- 完成匿名激光障碍保留、未知地图策略一致性、放置前实际可达性和抬升后新扫描检查；新增基础测试共 31 项通过，0.66 秒（核心 11、后端 9、启动 5、调度 6）。
- 后续故障用例及临时点修正后，新增基础测试共 35 项通过，0.68 秒（核心 11、后端 11、启动 5、调度 8）。
- 最终集成曾暴露临时点边界：计算 action 点与 A* 栅格终点不完全相同，实际停车位置会使临放超出臂长；系统回退复原成功并结束该轮。已将候选操作点按实际栅格量化、计入停车容差，默认临时点偏移由 0.85 m 改为 0.75 m；没有放宽臂长、控制器容差或 Safety Gate。测试期间新增用例曾加载旧安装副本，现已重建并串行进行最终全量回归。
- 最终联动测试已获准在允许本地 DDS 套接字的环境执行，仍将所有测试输出放在 `src/task`。相关 plan/controller 回归各出现一项既有失败：几何校验不支持 `arm_1_joint`；Safety Gate 测试在 0.4 m 期待停止，但现有配置的停止距离为 0.2 m。未修改这些范围外文件或保护阈值。
- **最终重建通过；task 全量测试通过，耗时 4 分 11 秒。** 共 64 个 pytest 用例（原有 23、新增 41），8/8 CTest 测试组通过；`colcon test-result` 汇总含测试组共 72 项，0 errors、0 failures、0 skipped。最终日志：`src/task/logs/final_build.log`、`final_task_tests.log`、`task_results.log`。
- plan 回归 11/12 CTest 组通过，controller 回归 9/10 组通过；上述既有失败的详细输出保存在 `src/task/logs/plan_results.log`、`controller_results.log`，没有将它们记录为通过。
- 最终 AST 检查通过（19 个 Python 文件），`git diff --check` 通过；原 task 点位、URDF、控制器配置和 description 四个受保护文件哈希均与开始时一致。
- 已实际尝试 `obstacle_recovery.launch.py gui:=false rviz:=false`：本地通信套接字被沙箱禁止；gzserver 因无法创建 `/home/dx/.gazebo/server-11385` 退出。已停止本次启动，未修改外部目录以绕过限制。
- Gazebo 启动失败日志：`src/task/logs/gazebo_acceptance_launch.log`。真实场景的定位、激光、物体移位闭环尚未通过；这是环境前提阻塞，不是自动化测试通过证明。
- **剩余事项：真实 Gazebo 四场景验收。** Gazebo 固定用户日志目录位于 `src/task` 外，现有授权不包含该写入；需要用户明确授权或提供已经兼容运行的 Gazebo，再验证地图对齐、原始激光变化、搬运与复原误差、后方遮挡再阻塞。当前不能宣称完整 Gazebo 验收完成。

## 构建与测试命令

在工作区根目录执行，先加载 `/opt/ros/humble/setup.bash` 和原 `install/setup.bash`，设置 `ROS_LOG_DIR=$PWD/src/task/logs/ros`、`TMPDIR=$PWD/src/task/logs/tmp`、`PYTHONDONTWRITEBYTECODE=1`。

```bash
colcon --log-base src/task/.colcon/log build \
  --build-base src/task/.colcon/build --install-base src/task/.colcon/install \
  --packages-select plan controller car_description task \
  --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
source src/task/.colcon/install/setup.bash
colcon --log-base src/task/.colcon/log test \
  --build-base src/task/.colcon/build --install-base src/task/.colcon/install \
  --packages-select task
colcon test-result --test-result-base src/task/.colcon/build --verbose
```

详细启动方式和模拟边界见 [OBSTACLE_RECOVERY.md](../src/task/OBSTACLE_RECOVERY.md)。

本轮基础测试实际使用源码 Python 路径及 task overlay：

```bash
PYTHONPATH="$PWD/src/task:$PYTHONPATH" python3 -m pytest \
  src/task/test/test_recovery_core.py src/task/test/test_gazebo_backend.py \
  src/task/test/test_recovery_launch.py src/task/test/test_recovery_node.py \
  -q -p no:cacheprovider --basetemp src/task/logs/tmp/recovery_unit
PYTHONPATH="$PWD/src/task:$PYTHONPATH" python3 -m pytest \
  src/task/test/test_node.py -q -p no:cacheprovider \
  --basetemp src/task/logs/tmp/legacy_nodes
PYTHONPATH="$PWD/src/task:$PYTHONPATH" python3 -m pytest \
  src/task/test/test_recovery_pipeline.py -xq -p no:cacheprovider \
  --basetemp src/task/logs/tmp/recovery_pipeline
```
