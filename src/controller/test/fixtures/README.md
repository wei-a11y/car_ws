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
