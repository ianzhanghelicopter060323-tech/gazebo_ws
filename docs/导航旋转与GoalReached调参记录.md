# 导航旋转与 Goal Reached 调参记录

更新时间：2026-07-29  
工作空间：`/home/ianichinose/gazebo_ws`

## 目标

- 机器人能够正常执行导航，而不是只完成全局规划。
- 终点朝向误差保持在 `yaw_goal_tolerance: 0.10 rad`（约 5.7°）以内。
- `move_base` action 返回状态 3，并输出 `Goal reached`。
- 狭窄通道中不能因足迹或膨胀设置过大而直接规划失败。
- 所有测试保留 rosbag、日志、动态参数和可回退文件。

## 回退点

- 初始调参前：
  `/home/ianichinose/gazebo_ws/.codex/nav_tuning_backups/20260728_2045_pre_heading_goalreach`
- 雷达自体回波修正与最终候选持久化前：
  `/home/ianichinose/gazebo_ws/.codex/nav_tuning_backups/20260728_2325_before_final_persist`
- 完全照搬成熟方案前（包含当前配置、成熟参考配置、URDF、launch 和 SHA256）：
  `/home/ianichinose/gazebo_ws/.codex/nav_tuning_backups/20260728_before_mature_exact_copy`

## 已定位的根因

### 1. 低角速度无法克服 Gazebo 底盘静摩擦

- DWA 的旧参数会给出约 `0.20–0.25 rad/s` 的角速度命令，但实际角位移几乎为零。
- 手动 `0.30 rad/s` 仍响应很弱；手动 `0.50 rad/s` 可以稳定转动。
- `min_vel_theta: 0.40`、`max_vel_theta: 0.60`、`acc_lim_theta: 5.0` 能稳定触发实际旋转。

### 2. 雷达碰撞网格造成自体回波

- 修正前 720 条激光中有 552 条小于 0.3 m，最近约 0.044 m；最近外部模型实际约 0.99 m。
- 局部代价地图机器人中心代价值达到 82，DWA 在终点报告 `Rotation cmd in collision`。
- 原因是 `laser_link` 的装饰网格同时作为 collision，Gazebo 射线命中雷达自身。
- 移除 `laser_link` 的 collision 后，0.10 m 和 0.15 m 内回波均降为 0，机器人中心代价值变为 0。

### 3. 默认 footprint padding 隐式扩大足迹

- 配置足迹虽为 `±0.10 m`，costmap 默认仍添加 `0.01 m` padding。
- 旋转包络半径因此约为 0.156 m，恰好碰到狭窄终点的致命栅格。
- 动态设为 `footprint_padding: 0.0` 后，同一失败位置重试成功并输出 `Goal reached`。

### 4. 仅收紧位置容差或降低最大角速度不可作为最终方案

- `max_vel_theta: 0.50`、`acc_lim_theta: 10.0` 去程成功，但返程持续 ACTIVE 并进入恢复。
- `xy_goal_tolerance: 0.05` 能修复单个终点朝向，但完整返程超时。
- 足迹从 `±0.10 m` 临时缩到 `±0.09 m` 仍可能在中段进入代价 99 区并被 DWA 连续拒绝。
- 单纯把 `occdist_scale` 提到 0.08、全局膨胀提到 0.15 m 会显著降低通过速度，并出现高频失败重规划。

## 关键实测结果

| 数据目录 / 用例 | 结果 | 位置误差 | 朝向误差 | 备注 |
|---|---:|---:|---:|---|
| `run02_min040_clean/case01_yaw030_clean` | 状态 3 | 0.0022 m | 0.091 rad | 原地 +0.3 rad |
| `run02_min040_clean/case02_yaw_minus030` | 状态 3 | 0.0031 m | 0.089 rad | 原地 -0.3 rad |
| `run02_min040_clean/case03_nav_x120_yaw0` | 状态 3 | 0.0929 m | 0.069 rad | 正常导航 |
| `run03_rotation_fix_passability/case01_narrow_branch_direct` | 状态 3 | 0.0952 m | 0.080 rad | 狭窄支路去程 |
| `run03_rotation_fix_passability/case02_return_origin_yaw0` | 状态 3 | 0.0841 m | 0.092 rad | 支路返程 |
| `run04_scan_self_echo_stage_a/case01_yaw030` | 状态 3 | 0.0015 m | 0.092 rad | 雷达修正后 |
| `run04_scan_self_echo_stage_a/case03_retry_branch_padding0` | 状态 3 | 0.0956 m | 0.099 rad | 同一失败点 padding A/B |
| `run04_scan_self_echo_stage_a/case04_return_origin_padding0` | 状态 3 | 0.0816 m | 0.096 rad | 无 DWA failed、清图或恢复 |
| `run05_final_persisted/case01_yaw030` | 状态 3 | 0.0010 m | 0.091 rad | 全新进程从文件加载 |
| `run05_final_persisted/case02_narrow_branch` | 状态 4 | 0.0867 m | 0.126 rad | 随机环境下终点旋转碰撞 |
| `run05_final_persisted/case03_retry_branch_xy005` | 状态 3 | 0.0473 m | 0.094 rad | 收紧位置容差只修复单点 |
| `run05_final_persisted/case04_return_xy005` | 超时 | 1.150 m | 3.097 rad | 完整返程不可接受 |

所有测试数据位于：

`/home/ianichinose/gazebo_ws/.codex/nav_tuning_runs`

## 完全照搬成熟方案的测试约束

参考目录：

`/home/ianichinose/car/src/gazebo_nav/config/move_base`

该方案的原始 launch 实际选择 `teb_local_planner/TebLocalPlannerROS`，并加载
`teb_local_planner_params.yaml` 与 `move_base_params.yaml`；DWA 文件不是其默认运行配置。
因此“完全照搬”测试应包含：

- 使用 TEB，而不是继续加载 DWA。
- 使用参考中的 `robot_radius: 0.05`。
- 膨胀半径 0.10 m、`cost_scaling_factor: 15.0`。
- Dijkstra 全局规划、梯度路径、`default_tolerance: 0.4`。
- `planner_frequency: 5.0`、`controller_frequency: 15`。
- 禁止 clearing rotation。

原样测试不得覆盖本文件所列回退点。若因当前机器人实际 TF 名称
`laser_link` 与参考中的 `laser_frame` 不同而无法生成 costmap，应记录为兼容性失败，
再进行最小适配测试。


## 最新现场验证基线

记录日期：2026-07-28

最新基线快照：

`/home/ianichinose/gazebo_ws/.codex/nav_tuning_backups/20260728_latest_front_nav_good_cone_contact`

现场验证结论：

- 当前版本在前置导航路段效果很好，保留为后续调参的最新基线。
- 进入锥桶区后，机器人会与锥桶发生实体接触并推动锥桶，说明当前避障安全裕量不足。
- 本次只记录并保存基线，不修改当前导航参数。
- 后续优先确认锥桶是否稳定进入 `local_costmap`；若感知正常，再以小步长适当增大
  `footprint`，并同时复测狭窄路段的全局规划、DWA 执行、终点旋转和
  `Goal reached`，避免重新引入 `plan failed`。
- 当前配置未显式设置 `footprint_padding`，使用 costmap 默认 padding；后续调整时应将
  footprint 与 padding 作为同一组安全边界评估。

## 2026-07-29 锥桶区非半径调参

### 约束与回退点

- 按要求保持膨胀半径不变：
  `global_inflation_radius: 0.10 m`、`local_inflation_radius: 0.15 m`。
- 本轮调参前完整快照：
  `/home/ianichinose/gazebo_ws/.codex/nav_tuning_backups/20260729_before_cone_margin_nonradius_tuning`
- 本轮通过验证后的最新基线：
  `/home/ianichinose/gazebo_ws/.codex/nav_tuning_backups/20260729_latest_cone_margin_nonradius_verified`
- 完整测试数据：
  `/home/ianichinose/gazebo_ws/.codex/nav_tuning_runs/run07_cone_margin_nonradius`

### 最终保留参数

- 保留现场已修改的 `±0.12 m` 方形 footprint，并显式设置
  `footprint_padding: 0.01 m`。
- 不改变膨胀半径，仅将 `cost_scaling_factor` 从 `3.5` 调整为 `2.8`，
  使有限膨胀范围内的代价衰减更慢。
- DWA：
  - `path_distance_bias: 30.0 -> 26.0`
  - `occdist_scale: 0.03 -> 0.04`
  - `stop_time_buffer: 0.10 -> 0.15`
  - `scaling_speed: 0.80 -> 0.65`
  - `goal_distance_bias: 20.0` 和 `max_scaling_factor: 0.20` 保持不变。
- `local_costmap/update_frequency: 6.0 -> 10.0 Hz`。
- `GlobalPlanner/use_grid_path` 保持 `true`。

### 已否决的参数

- `use_grid_path: false` 在当前地图上触发大量
  `NO PATH / Failed to get a plan from potential when a legal potential was found`，
  机器人中途失去全局路径，因此恢复为 `true`。
- `scaling_speed: 0.50` 与 `stop_time_buffer: 0.20` 组合过于保守，
  在进入锥桶区前的固定障碍路段就多次出现 `DWA planner failed to produce path`，
  因此回调为 `0.65/0.15`。
- 调参前运行时参数在同一布局 A/B 测试中推进距离更短，说明全程人工目标的前段停滞
  不是本轮避障权重单独造成。

### 最终验证结果

| 用例 | 结果 | 位置误差 | 朝向误差 | 锥桶位移 |
|---|---:|---:|---:|---:|
| `case05_cone_entrance_local_avoidance` | 状态 3，`Goal reached` | 0.1007 m | 0.0886 rad | 10 个均为 0 |
| `case06_front_nav_regression` | 状态 3，`Goal reached` | 0.0897 m | 0.0906 rad | 10 个均为 0 |

`case05` 从锥桶区入口 `(0,-1.0,-π/2)` 导航到 `(-0.5,-2.0,-π/2)`，
路径长度约 `1.17 m`；机器人到锥桶中心的实测最小距离为 `0.4587 m`，
没有锥桶移动超过 `1 cm`。该用例证明当前参数可以在不增加膨胀半径的前提下完成
锥桶区局部避障。

`case06` 从原点导航到 `(1.2,0,0)`，用于确认前置导航和终点判定没有退化。

注意：从原点直接发送锥桶区深处目标的人工全程用例，会先经过前置固定方块和地图狭窄
区域，并在进入锥桶区前出现 DWA 停滞。因此本轮已验证“前置导航”和“锥桶区局部避障”
两个分段用例，但尚未把该人工全程目标标记为端到端通过；后续仍应以实际自动导航路线
进行复核。
