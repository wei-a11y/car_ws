# 预瞄与速度滞后回放

三个 CSV 是本次隔离 Gazebo 开发探针的固定输入，只用于控制器回归测试。
位置和航向来自 TF 定位估计，速度来自 `/odom`，不是 Gazebo 真值。

- `preview_budget_path.csv`：`20261028_211647_916924_gazebo/combined_outward_1`
  的 `planned_path.csv`，保留关联路径、曲率及区间证书。
- `preview_budget_inputs.csv`：同次记录的 `actual_path.csv`，仅保留
  `x,y,yaw,v_actual,w_actual`。
- `preview_budget_lag_inputs.csv`：`20261028_212716_170876_gazebo/combined_outward_1`
  的相同五列，用于覆盖测量速度滞后。

每组输入都从新建的控制器开始回放。测试固定物理参数，不修改原始实验记录。
该目录使用 `fixtures`，避免被仓库的通用 `data/` 忽略规则排除。

## 实车雷达扫描回归

`real_lidar.scan` 来自仓库根目录保留的 `scan_once.yaml`，原始时间戳为
`1793713442.836489726`，frame 为 `laser_link`。只转换数据格式，不修改
720 个距离值；正无穷写为 `inf`。第一行依次为 `angle_min angle_max
angle_increment range_min range_max scan_yaw`，其后每行一个距离值。
`scan_yaw=3.14` 取自当前原 URDF 的 `laser_joint`，不是实测安装标定。

按该旋转计算，前方 ±45° 共 179 条射线，147 条有限有效、32 条正无穷；
最大有限回波间隔为约 0.087145 rad。测试比较严格策略和有限缺失策略，
验证数据处理行为，不证明 0.80/0.105 的阈值已经适合实车。
该夹具仅供离线核心测试，不将历史扫描时间戳作为实时有效消息发布。
