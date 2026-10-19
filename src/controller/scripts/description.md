# ROS 2 DWA 与 LQR 路径跟踪记录和性能评价说明

## 1. 文档目的

本文档说明如何使用两个 Python 程序记录并分析 ROS 2 移动机器人在 **DWA 控制器**和 **LQR 控制器**下的路径跟踪表现：

1. `PathRecorder`：订阅规划路径 `/plan`，通过 TF 定时记录机器人实际位姿，并将规划路径和实际轨迹保存为 CSV。
2. `Tracking Analysis`：读取规划路径和实际轨迹，计算实际轨迹到规划折线的最近距离、统计误差指标、计算轨迹长度，并生成可视化图片。

这套工具对控制器本身保持独立。只要 DWA 或 LQR 使用同一个规划路径接口，机器人实际位姿也能通过配置的 TF 查到，就可以用同一套工具记录数据。**记录和分析脚本不直接控制机器人，也不会自动调整 DWA 或 LQR 的参数。**

建议把两种控制器的实验数据分别保存，再在相同地图、起点、目标点和速度/安全限制条件下进行比较。

---

## 2. 系统关系与数据流

```text
                        A* / Phase 4
                             |
                             | /plan
                             | nav_msgs/msg/Path
                             v
                    +-------------------+
                    |    PathRecorder   |
                    |                   |
                    | 记录 /plan        |
                    | 查询 map -> base  |
                    | 定时保存实际位姿  |
                    +---------+---------+
                              |
                              v
                   planned_path.csv
                   actual_path.csv
                              |
                              v
                    Tracking Analysis
                              |
                 +------------+------------+
                 |                         |
                 v                         v
           tracking_error.csv       PNG 可视化图

控制器可分别选择：

  /plan -> DWA -> /cmd_vel_raw -> Safety Gate -> 底盘
  /plan -> LQR -> /cmd_vel_raw -> Safety Gate -> 底盘

PathRecorder 不依赖 DWA 或 LQR 的内部实现；它记录的是同一规划路径接口和机器人实际 TF 轨迹。
```

> 测试时不要让 DWA 与 LQR 同时向同一底盘发送控制指令。每次运行只启用一个控制器，并保持相同的 Safety Gate 和底盘控制链路。

---

## 3. 组成文件和依赖

| 程序 | 用途 | 运行方式 |
|---|---|---|
| `path_recorder.py`（示例文件名） | 在线记录规划路径、机器人实际轨迹 | ROS 2 Python 节点 |
| `tracking_analysis.py`（示例文件名） | 离线计算跟踪误差、路径长度并绘图 | 普通 Python 程序 |
| `description.md` | 本模块的使用与实现说明 | 文档，不需要编译 |

实际文件名和 ROS 2 包名请以你的工作空间为准。

### 3.1 路径记录节点依赖

- ROS 2 `rclpy`
- `nav_msgs`
- `tf2_ros`

### 3.2 离线分析脚本依赖

- Python 3
- `numpy`
- `pandas`
- `matplotlib`

可在运行分析脚本的 Python 环境中安装：

```bash
python3 -m pip install numpy pandas matplotlib
```

如果系统使用 ROS 2 自带的 Python 环境或虚拟环境，请确保这些库安装在实际运行脚本的同一环境中。

---

## 4. PathRecorder 路径记录节点

### 4.1 节点名称

```text
path_recorder
```

### 4.2 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `plan_topic` | `/plan` | 规划路径话题，类型为 `nav_msgs/msg/Path` |
| `global_frame` | `map` | 查询机器人位姿使用的全局坐标系 |
| `base_frame` | `base_footprint` | 机器人基座坐标系 |
| `record_period` | `0.05` | 记录周期，单位为秒，默认约 20 Hz |
| `output_dir` | `data/path_record` | CSV 保存目录；相对路径以节点进程的当前工作目录为基准 |

### 4.3 规划路径输入

节点订阅：

```text
Topic: /plan
Type:  nav_msgs/msg/Path
```

接收到非空 `Path` 消息后，节点遍历 `msg.poses`，提取每个姿态的 `position.x` 和 `position.y`，并保存为二维路径点。当前代码不保存规划点的姿态、时间戳、速度或控制器名称。

规划路径文件：

```text
planned_path.csv
```

格式：

```csv
index,x,y
0,-10.800000,-5.900000
1,-10.500000,-5.800000
2,-10.200000,-5.600000
```

字段说明：

- `index`：路径点序号，从 0 开始。
- `x`、`y`：规划路径在相应全局坐标系中的位置，单位为米。

**重要：** 每次接收到新的非空 `/plan`，当前实现都会覆盖写入 `planned_path.csv`。如果导航期间规划路径不断更新，最终文件通常只保留最近一次收到的路径，不会保存每个历史版本。

### 4.4 机器人实际位姿输入

节点通过 TF 查询：

```text
map -> base_footprint
```

核心调用：

```python
self.tf_buffer.lookup_transform(
    self.global_frame,
    self.base_frame,
    rclpy.time.Time()
)
```

查到最新可用变换后，记录机器人位置 `x`、`y` 和航向角 `yaw`。如果 TF 查询失败，节点输出警告并跳过该次采样。

规划路径坐标必须与实际轨迹坐标处于同一坐标系。默认配置假设 `/plan` 的点位于 `map` 坐标系，且 TF 为 `map -> base_footprint`。如果 `/plan` 的 `header.frame_id` 不是 `map`，应先确认是否需要做坐标变换；当前代码没有根据每条路径消息的 `header.frame_id` 自动转换路径点。

### 4.5 记录时间

第一次成功查询 TF 时，节点记录 `start_time`。之后的相对时间为：

```text
time = 当前 ROS 时钟 - start_time
```

时间单位为秒，保留 6 位小数。它不是导航目标发送时间，也不是规划路径的时间戳。若仿真使用 `/clock` 和 `use_sim_time`，节点的计时行为会随 ROS 时钟配置而定。

### 4.6 yaw 计算

TF 姿态用四元数表示。程序通过以下公式计算二维航向角：

```python
siny_cosp = 2.0 * (w * z + x * y)
cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
yaw = math.atan2(siny_cosp, cosy_cosp)
```

`yaw` 的单位是弧度（rad）。

### 4.7 实际轨迹文件

输出文件：

```text
actual_path.csv
```

格式：

```csv
time,x,y,yaw
0.000000,-10.833000,-5.900000,-1.948000
0.050000,-10.830000,-5.895000,-1.940000
0.100000,-10.825000,-5.887000,-1.930000
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `time` | 相对记录起始时间 | s |
| `x` | 机器人全局 X 坐标 | m |
| `y` | 机器人全局 Y 坐标 | m |
| `yaw` | 机器人平面航向角 | rad |

节点启动时以写入模式打开文件，所以新运行会覆盖之前的 `actual_path.csv`。正常关闭节点时，`destroy_node()` 会关闭文件。

### 4.8 输出目录注意事项

默认路径为：

```text
data/path_record/
```

这是相对路径，位置取决于启动节点时进程的当前工作目录。建议明确指定绝对路径，或者始终从同一项目目录启动节点，避免找不到 CSV 文件。

---

## 5. Tracking Analysis 离线分析脚本

### 5.1 输入文件

分析脚本当前默认读取：

```text
data/path_record/planned_path.csv
data/path_record/actual_path.csv
```

规划路径 CSV 至少需要 `x`、`y` 两列；实际轨迹 CSV 需要 `time`、`x`、`y` 列。`yaw` 列由记录节点生成，但当前路径误差计算不使用它。

脚本检查规划路径至少有两个点、实际轨迹不能为空。当前实现假设文件存在、列名正确且数值有效；不满足时需要根据错误提示检查数据。

### 5.2 点到线段距离

函数：

```python
point_to_segment_distance(px, py, ax, ay, bx, by)
```

计算实际点 `P=(px, py)` 到规划线段 `AB` 的最短欧氏距离。

计算步骤：

1. 计算线段向量 `AB=(bx-ax, by-ay)` 和向量 `AP=(px-ax, py-ay)`。
2. 计算投影比例：

   ```text
t = (AP · AB) / |AB|²
```

3. 将投影比例限制在 `[0, 1]`：

   ```text
t = max(0, min(1, t))
```

4. 用比例 `t` 计算线段上的最近投影点。
5. 计算实际点与投影点之间的欧氏距离。

如果 A、B 两点重合，代码直接计算实际点到 A 的距离，避免除以零。

这里的 `t` 限制在 `[0, 1]`，是线段投影比例的截断，不是对整套误差指标进行标准化。

### 5.3 跟踪误差（Cross-Track Error）

函数：

```python
calculate_cross_track_error(actual_x, actual_y, planned_xy)
```

对实际轨迹中的每个点，遍历整条规划折线的所有线段，计算该实际点到各线段的距离，并取最小值：

```text
error_i = min(distance(actual_point_i, planned_segment_j))
```

输出的 `errors` 数组与实际轨迹点数相同，单位为米（m）。

该指标的准确含义是：**实际机器人位置到整条规划折线的最近几何距离**。它不要求实际轨迹点和规划路径点在时间或索引上逐一对应，也不直接反映沿路径的进度误差。

如果路线有回折、相邻通道很近或路径段空间上接近，实际位置可能匹配到规划路径中另一段线段。因此，该指标适合评价整体几何偏离程度，但在复杂路线中并不总等同于控制器当前跟踪目标处的横向误差。

### 5.4 误差输出 CSV

程序向实际轨迹 DataFrame 添加 `tracking_error` 列，再保存为：

```text
data/path_record/tracking_error.csv
```

例如：

```csv
time,x,y,yaw,tracking_error
0.000000,-10.833000,-5.900000,-1.948000,0.012000
0.050000,-10.830000,-5.895000,-1.940000,0.015000
```

### 5.5 统计指标

分析程序计算以下指标：

| 指标 | 计算方法 | 含义 |
|---|---|---|
| MAE | `mean(abs(error))` | 平均最近路径距离 |
| RMSE | `sqrt(mean(error**2))` | 对较大误差更敏感的综合指标 |
| Max Error | `max(error)` | 最大观测偏差 |
| Median | `median(error)` | 误差中位数 |
| P95 | `percentile(error, 95)` | 95% 分位误差 |
| 超阈值比例 | `count(error > threshold) / N * 100%` | 误差超过阈值的样本比例 |

当前默认阈值为：

```python
threshold = 0.10
```

即 0.10 m（10 cm）。它只影响统计输出和绘图，不会改变 DWA 或 LQR 的控制行为。

### 5.6 路径长度

规划路径长度是规划折线上相邻路径点之间距离的总和：

```text
planned_length = sum(distance(planned[i], planned[i+1]))
```

实际轨迹长度是实际轨迹相邻采样位置之间距离的总和：

```text
actual_length = sum(distance(actual[i], actual[i+1]))
```

长度差：

```text
length_error = actual_length - planned_length
```

实际长度更大可能与摆动、额外修正或绕行有关，但仅凭长度差无法确定原因。两种长度也会受到规划路径后处理、路径点密度、采样周期以及导航是否完整到达目标的影响。

### 5.7 生成的图片

**`trajectory_comparison.png`**：绘制规划路径、实际轨迹、规划起点和终点。`plt.axis('equal')` 用于保持 X、Y 轴比例一致。

**`tracking_error.png`**：横轴是实际轨迹的 `time`（秒），纵轴是跟踪误差（米），并绘制默认 0.10 m 阈值线。

当前脚本不会自动识别弯道区段、不会区分直线误差与弯道误差，也不会标记速度为零的时刻。

### 5.8 默认输出文件

```text
data/path_record/
├── planned_path.csv
├── actual_path.csv
├── tracking_error.csv
├── trajectory_comparison.png
└── tracking_error.png
```

前两个 CSV 由 PathRecorder 生成，后三个结果由分析脚本生成。

---

## 6. DWA 和 LQR 的共同评价方式

### 6.1 记录节点不区分控制器

PathRecorder 只读取 `/plan` 和 TF 中的机器人实际位姿。因此，它可以用于 DWA，也可以用于 LQR；无需因为控制器名称不同而修改路径误差的数学计算。

使用时应确保：

- 每次实验只有一个控制器实际控制底盘。
- `/plan` 是该次实验的规划路径。
- TF 使用同一套正确的全局坐标系和机器人基座坐标系。
- DWA 与 LQR 采用相同地图、起点、目标点和相同的安全限制条件。
- 每次实验的 CSV 单独保存，避免文件互相覆盖。

### 6.2 单独运行 DWA 实验

1. 启动导航系统并选择 DWA 控制器。
2. 启动 PathRecorder，将 `output_dir` 指向 DWA 专用目录，例如：

   ```text
   /home/xinxinzi/cars/car/car_ws/data/path_record/dwa
   ```

3. 在 RViz 设置初始位姿并发送目标点。
4. 等待实验结束，使用 `Ctrl+C` 正常停止记录节点。
5. 保留该目录中的 `planned_path.csv` 和 `actual_path.csv`。
6. 对这组数据运行 Tracking Analysis。

### 6.3 单独运行 LQR 实验

按照相同流程，只将控制器切换为 LQR，并把 `output_dir` 指向另一个目录，例如：

```text
/home/xinxinzi/cars/car/car_ws/data/path_record/lqr
```

同样记录规划路径和实际轨迹。不要将 LQR 的 CSV 写入 DWA 的数据目录，否则文件会被覆盖，或导致分析时把不同实验的数据混在一起。

### 6.4 当前分析脚本的目录配置

你提供的分析脚本把目录写死为：

```python
data_dir = Path('data/path_record')
```

因此，若数据分别保存在 `data/path_record/dwa/` 和 `data/path_record/lqr/`，分析某组数据时，需要把 `data_dir` 临时改为对应目录，或将所选实验的两个输入 CSV 放到脚本所使用的目录。每次运行后，输出的误差 CSV 与图片也会写到该 `data_dir`。

更便于重复实验的后续改进方式，是把 `data_dir` 做成命令行参数；但当前这段分析脚本还没有实现该参数。

---

## 7. DWA 与 LQR 的公平对比实验建议

为了避免把环境或输入差异误判成控制器性能差异，建议按下表控制实验条件：

| 条件 | DWA 实验与 LQR 实验应如何处理 |
|---|---|
| 地图和障碍物 | 相同 |
| 规划器及 Phase 4 后处理 | 相同 |
| 起点与起始朝向 | 尽可能相同 |
| 目标点 | 相同 |
| 全局坐标系与 TF | 相同 |
| 底盘和 Safety Gate | 相同 |
| 记录周期 | 相同，例如 0.05 s |
| 控制器 | 一次只启用 DWA 或 LQR 其中一个 |
| 数据目录 | 分开保存 |
| 测试次数 | 建议每种控制器重复多次 |

建议至少比较以下指标：

| 指标 | 来源 | 解释 |
|---|---|---|
| MAE | `tracking_error.csv` 的误差列 | 平均最近路径距离 |
| RMSE | 误差列 | 较大偏差的综合影响 |
| 最大误差 | 误差列 | 观测到的最差偏离 |
| P95 | 误差列 | 大部分轨迹点的偏差水平 |
| 超过 0.10 m 的比例 | 误差列 | 偏离超过阈值的样本占比 |
| 实际路径长度 | `actual_path.csv` | 实际运动距离 |
| 规划路径长度 | `planned_path.csv` | 规划折线长度 |
| 导航是否成功 | 需结合导航状态或日志判断 | 仅靠当前 CSV 无法可靠判定是否到达目标 |

可以创建一张结果汇总表：

| 控制器 | MAE (m) | RMSE (m) | Max Error (m) | P95 (m) | 超阈值比例 (%) | 实际路径长度 (m) |
|---|---:|---:|---:|---:|---:|---:|
| DWA | | | | | | |
| LQR | | | | | | |

不要只凭一张轨迹图或单次实验下结论。若要比较可靠，建议使用相同条件重复多次，并报告平均值及波动情况。

---

## 8. 当前代码的限制与解释注意事项

1. **当前没有记录控制器速度命令。** `actual_path.csv` 只保存 `time,x,y,yaw`，所以不能直接从现有 CSV 判断 `/cmd_vel_raw` 中 `linear.x` 是否变成 0，也不能量化角速度变化。如果需要分析 DWA/LQR 的速度平滑性，需要额外记录 `/cmd_vel_raw` 或最终 `/cmd_vel`，并注明记录的是哪个话题。
2. **当前不自动标记控制器。** CSV 中没有 `controller` 字段。通过将 DWA 与 LQR 保存到不同目录，手动区分实验。
3. **新路径会覆盖旧路径。** 每次收到非空 `/plan`，当前记录节点就覆盖 `planned_path.csv`。若需要保留全程每一次规划路径变化，必须额外实现带时间或版本号的历史记录。
4. **实际轨迹文件在节点启动时会覆盖。** 每个实验请使用独立输出目录，或提前备份旧数据。
5. **计时从首次成功 TF 采样开始。** 不等于目标发送时刻；如果要比较总导航时间，应从导航状态或目标发送事件单独记录。
6. **误差是到整条路径的最近几何距离。** 在回折路径或相近走廊中可能匹配到不对应的路径段。
7. **当前没有自动分离弯道和直线区域。** 整体 MAE/RMSE 可能掩盖局部弯道问题。
8. **当前没有自动判断 `v=0` 的原因。** 要区分 DWA/LQR 输出零速度、Safety Gate 清零或底盘未执行命令，需要同步记录 `/cmd_vel_raw`、`/cmd_vel` 和相关状态日志。
9. **路径坐标系必须匹配。** 默认假设规划点与 TF 查询位置都在 `map` 坐标系；程序没有根据 `/plan.header.frame_id` 自动转换路径坐标。
10. **长度和误差依赖记录完整性。** 若机器人中途停止、日志中断、路径未到达目标或 TF 缺样，统计值只描述记录到的数据，不一定代表一次完整成功的导航。

---

## 9. 推荐的完整操作流程

### 步骤 1：确认接口

确认 `/plan` 存在且类型是 `nav_msgs/msg/Path`，并确认 TF `map -> base_footprint` 可查询：

```bash
ros2 topic info /plan
ros2 run tf2_ros tf2_echo map base_footprint
```

第二条命令用于检查 TF。若项目使用不同的帧名，请按实际配置替换。

### 步骤 2：准备单独的数据目录

例如：

```text
car_ws/data/path_record/
├── dwa/
└── lqr/
```

为每个控制器选择各自目录。

### 步骤 3：运行一个控制器并记录

每次只启用一个控制器。启动 PathRecorder 时把 `output_dir` 设置为对应控制器的绝对路径。具体的 `ros2 run` 命令取决于该脚本安装到哪个 ROS 2 包中，可按项目实际包名运行。

### 步骤 4：完成实验并保存 CSV

导航测试后，正常停止记录节点，确认目录内存在：

```text
planned_path.csv
actual_path.csv
```

### 步骤 5：分析该控制器的数据

在分析脚本中将：

```python
data_dir = Path('data/path_record')
```

改成该次实验目录，例如：

```python
data_dir = Path('data/path_record/dwa')
```

分析 LQR 数据时改成 `data/path_record/lqr`。然后运行脚本。

### 步骤 6：填写比较表

记录各自 MAE、RMSE、最大误差、P95、超阈值比例和实际路径长度。如果要研究速度平滑性，需额外增加速度话题记录，而不能只依靠当前位置轨迹推断速度命令。

---

## 10. 输出文件总结

每次控制器实验目录建议包含：

```text
<experiment_dir>/
├── planned_path.csv
├── actual_path.csv
├── tracking_error.csv
├── trajectory_comparison.png
└── tracking_error.png
```

说明：

- `planned_path.csv`：该目录最后一次收到的规划路径。
- `actual_path.csv`：该次运行记录到的机器人实际位姿序列。
- `tracking_error.csv`：实际位姿数据加上跟踪误差列。
- `trajectory_comparison.png`：规划路径与实际轨迹对比图。
- `tracking_error.png`：跟踪误差随记录时间变化的曲线。

---

## 11. 总结

本模块是用于 **DWA 和 LQR 路径跟踪表现对比的通用离线评价工具**。它通过 `/plan` 获得规划折线，通过 `map -> base_footprint` TF 记录实际机器人位姿，再计算实际轨迹到规划路径的最近距离并生成统计和可视化结果。

当前实现可以用于比较两种控制器的几何路径跟踪精度，但还不能直接评价速度命令是否平滑，也不能单独识别弯道误差。若要分析“转弯时速度是否变成 0”“角速度是否突变”以及 Safety Gate 是否修改了命令，需要进一步记录 `/cmd_vel_raw` 和 `/cmd_vel`，并按控制器分别保存数据。
