# TEB_test `/move_base` 配置说明

活动规划器为 `global_planner/GlobalPlanner` 与
`teb_local_planner/TebLocalPlannerROS`。全局规划和 costmap 基线来自 `~/car`，TEB
局部参数则以本项目稳定 DWA 的运动限制为基线，使用 TEB 原生参数重新配置。

## 几何模型

为了同时保留全局可规划性和局部碰撞真实性，两个 costmap 不再共用机器人几何：

- global costmap：`robot_radius: 0.05 m`，用于维持已验证的全局路径形成能力；
- local costmap：`±0.08 m` 方形 footprint，加 `0.01 m` padding，有效轴向半宽
  `0.09 m`；
- TEB：使用同一组 `±0.08 m` polygon，并设置 `min_obstacle_dist: 0.02 m`，有效
  硬约束约为 `0.10 m`，略大于 local costmap 的最终碰撞模型；
- global costmap 保持 `inflation_radius: 0.20 m`，local costmap 单独使用
  `0.18 m`；二者的 `cost_scaling_factor` 均为 `15.0`。

这种拆分允许 GlobalPlanner 生成路径，但局部 costmap 的最终可行性检查和 TEB
优化使用一致的几何边界。当前局部边界有意比原 DWA 的 `0.144 m` 放宽；规则允许轻微
擦碰后继续导航，因此优先避免机器人因局部禁区过宽而长期卡死。若全局路径经过物理上
确实不可通行的空隙，TEB 仍应拒绝执行，而不是穿过障碍。

## TEB 基线

- DWA 的 `max_vel_trans: 1.25 m/s` 在 TEB 0.9.1 中没有对应参数，因此把 x/y 上限
  均设为 `0.9 m/s`，最大对角速度约 `1.27 m/s`；
- 加速度和角速度沿用 DWA：`acc_lim_x/y: 1.3`、`acc_lim_theta: 4.0`、
  `max_vel_theta: 1.6`；
- `dt_ref: 0.10`、`min_samples: 5`，保留较密的轨迹离散；
- `feasibility_check_no_poses: 5`，只硬检查近期轨迹，避免远端转弯尚未优化完成时
  连续拒绝当前控制；
- `global_plan_viapoint_sep: 0.10`、`weight_viapoint: 20`，使局部轨迹跟随全局路径，
  同时保留绕开锥桶的能力；
- `weight_optimaltime: 1`、`weight_obstacle: 100`，避免时间最优项驱使轨迹抄近路；
- `free_goal_vel: false`，与 DWA 的停车式到达行为一致。
- `xy_goal_tolerance: 0.04 m` 保持位置精度；`yaw_goal_tolerance: 0.15 rad`
  允许中间点在约 8.6° 内结束，避免已到位置后反复原地调整朝向。
- move_base 振荡看门狗使用 `10 s / 0.05 m`；短时间原地调整终点朝向不会再被
  `4 s / 0.3 m` 条件误判为卡死。

`min_vel_x/y`、`max/min_vel_trans`、`min_vel_theta` 和 `stopped_vel` 是 DWA 参数，
TEB 0.9.1 不读取，故未放入 TEB YAML。

## 加载与验证

膨胀参数只由本目录 YAML 负责：`costmap_common_params.yaml` 提供 global 的 `0.20 m`
基线，`local_costmap_params.yaml` 将 local 覆盖为 `0.18 m`，launch 文件不再覆盖。
`gazebo_nav.launch` 会在启动 `move_base` 前清空 `/move_base` 参数命名空间，防止旧
footprint/radius 残留。修改 YAML 后必须至少重启 `move_base`；正在运行的节点不会自动
采用新参数。

`smart_factory_bringup/full_competition.launch` 通过 `simulation.launch` 复用同一个
`gazebo_nav.launch`，因此完整任务和单独导航加载完全相同的 TEB/代价地图参数。任务层
只通过 `/smart_factory/navigation` 向 `/move_base` 发目标，不在运行中切换规划器 profile。

## 2026-08-13 GUI/RViz 手动验证

在全新 Gazebo、RViz 与 move_base 实例中，按前序路线连续发送 seq1、seq9、seq10、
seq11、seq15、seq18、seq20、seq22、seq25、seq29、seq34、seq35，共 12 个原生
move_base 目标，结果为 12/12 `SUCCEEDED`。导航期间记录到 1 次 seq15→seq18 的瞬时
`trajectory is not feasible` WARN，随后无需清图即到达；ERROR、导航中止和 NO PATH 均为
0。此前在 seq25 复现的终点朝向振荡，在 `yaw_goal_tolerance: 0.15` 下约 3.4 秒到达，
未再触发 oscillation recovery。
