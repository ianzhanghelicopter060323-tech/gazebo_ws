# Adaptive TEB 锥桶区导航 Handoff

> 更新时间：2026-08-15（Asia/Shanghai）  
> 工作区：`/home/ianichinose/gazebo_ws`  
> 分支：`TEB_test`  
> 基准 HEAD：`ddcb359` (`nvigation already good enough`)  
> 当前状态：按用户要求暂停调参，没有 Gazebo、RViz、ROS 或压力测试进程在运行。

## 1. 目标与验收标准

目标是在不破坏前序导航稳定性的前提下，提高锥桶区宽路选择和避障能力。
当前定向验收标准为：

- `source_round_009` 端到端连续 3 轮完赛；
- 3 轮均无锥桶碰撞、卡死、导航失败或识别失效；
- GUI 和 RViz 可见，压力测试脚本保存可复查证据。

当前连续计数是 **`0/3`**。最新一轮完赛，但碰撞了 `cone_18`，不能计入通过样本。

前序导航已有 100 轮独立稳定性证据：`100/100` 导航成功、0 失败。该测试按当时需求
不强制终点朝向精度。证据见
[`pre_navigation_trials_20260813_114828_ros1132/summary.json`](../script/logs/pre_navigation_trials_20260813_114828_ros1132/summary.json)。

## 2. 合规边界

以 [`docs/simu_requirement.md`](simu_requirement.md) 的“补充答疑”为最新规则：

- 全局规划仅使用官方 GlobalPlanner，不修改、拦截、覆盖或重发全局路径；
- 局部动态运动规划/避障允许 TEB 或自研逻辑；
- 局部决策必须以实时激光雷达为核心输入；
- 规则明确允许发布中间坐标。

当前实现只读取官方全局路径和 `/scan`，不改写全局路径。动态 selector 使用 `/scan`
生成候选，通过官方 `/move_base/make_plan` 检查可达性，再发布普通中间 goal。
Gazebo world state 只由压力测试监控器读取，用于碰撞/录像证据，不输入导航算法。

后续不得向 selector 写入锥桶真值坐标或固定赛道轨迹，也不得绕过 `/scan` 作场景特判。

## 3. 系统结构

```text
任务目标
  └─→ 准备位姿 2
       └─→ 激光扇形候选 + 官方 make_plan 筛选
            └─→ 0..N 个滚动中间 goal
                 └─→ handoff 给 avoidance HCP-TEB 完成车间目标

官方 GlobalPlanner 路径
  └─→ AdaptiveTebLocalPlannerROS
       ├─→ baseline：小 footprint、单拓扑 TEB，前序导航
       └─→ avoidance：大 footprint、多拓扑 HCP-TEB，锥桶区
```

### 3.1 Adaptive TEB

`smart_factory_adaptive_teb/AdaptiveTebLocalPlannerROS` 初始化两个官方 `TebLocalPlannerROS`：

- `baseline`：单拓扑，`0.16 m` 方形 footprint；
- `avoidance`：多拓扑，`0.30 m` 方形 footprint；
- C++ selector 从 `/scan` 识别紧凑障碍点簇，计算障碍到官方路径的净空；
- 进入/退出采用多帧滞回，避免单帧噪声引起切换；
- 动态通道阶段通过 `set_avoidance_lock` 锁定 avoidance，避免中间 goal 交接时切回小 footprint。

avoidance TEB 不可行时会停止并报错，**不会**回退到小 footprint baseline 强行通过。
baseline 在自身不可行且有激光障碍证据时仍可请求 avoidance 恢复；这是由小几何向大几何
切换，不是 avoidance 缩小几何的 fallback。

### 3.2 滚动中间点

1. 候选区是以当前机器人为顶点的扇形，不是固定地图点。
2. 第一轮扇形中轴为准备位姿 2 的入口朝向，防止选点退回前序走廊。
3. 后续扇形每轮以当前位置更新，中轴转向车间目标，但始终受入口深度下限约束。
4. 对每个候选调用官方 `make_plan`，用激光点评估进入路径和后续路径短前缀的净空。
5. 评分以路径瓶颈净空为主，路长和扇形中轴偏离只是小惩罚。
6. 满足最少点数、入口深度和后续净空后，handoff 给普通 avoidance HCP-TEB。

已发布的 goal 在 action 期间保持固定；到达、超时或 handoff 超时时才重新生成扇形。

### 3.3 超时、回退与重选

- 滚动点墙钟超时 `45 s`；handoff 超时 `88 s`；回退超时 `45 s`；
- 失败点周围 `0.30 m` 禁止再选，最多重选 2 次；
- 已有成功动态点时，先回退到上一确认点再重选；
- 第一动态点超时时不回退准备位姿 2，而是从当前位置重选；
- 只对超时强制重选，允许从最低 `0.16 m` 的已继承起步瓶颈脱困，但不得恶化，
  且必须在 `0.25 m` 内恢复到 `0.20 m` 硬净空；
- 正常选点仍使用 `0.198 m` 起步逃逸下限和 `0.20 m` 硬恢复下限；
- 没有安全候选时 fail-closed，不缩小 footprint 或忽略净空。

## 4. 暂停时实际参数

### 4.1 Adaptive selector

```yaml
compact_cluster_max_span: 0.45
plan_lookahead_distance: 1.20
corridor_half_width: 0.30
two_cluster_clearance: 0.35
one_cluster_clearance: 0.30
entry_window_frames: 8
entry_required_frames: 5
exit_clear_frames: 20
minimum_avoidance_duration: 2.5
baseline_cooldown: 2.0
```

`compact_cluster_max_span: 0.45` 用于排除前序走廊长墙点簇，没有新的前序误触发证据时不建议修改。

### 4.2 Baseline TEB

```yaml
max_vel_x: 0.90
max_vel_y: 0.90
max_vel_theta: 1.60
acc_lim_x: 1.30
acc_lim_y: 1.30
acc_lim_theta: 4.00
footprint: [[0.08,-0.08], [0.08,0.08], [-0.08,0.08], [-0.08,-0.08]]
min_obstacle_dist: 0.02
inflation_dist: 0.08
yaw_goal_tolerance: 0.08
enable_homotopy_class_planning: false
```

baseline 是前序稳定性基线，非确有前序失败证据不要改动。

### 4.3 Avoidance TEB

```yaml
max_vel_x: 0.55
max_vel_y: 0.35
max_vel_theta: 1.60
acc_lim_x: 0.90
acc_lim_y: 0.70
acc_lim_theta: 4.00
footprint: [[0.15,-0.15], [0.15,0.15], [-0.15,0.15], [-0.15,-0.15]]
min_obstacle_dist: 0.10
inflation_dist: 0.16
weight_inflation: 2.0
weight_viapoint: 10.0
enable_homotopy_class_planning: true
max_number_classes: 3
selection_obst_cost_scale: 15.0
selection_viapoint_cost_scale: 0.2
selection_prefer_initial_plan: 0.95
selection_cost_hysteresis: 0.85
switching_blocking_period: 4.0
```

不建议继续降低 `max_vel_y`。入口需要麦克纳姆底盘横移；转向只能通过提前调姿减少横向
扫碰，不能完全替代横移。角速度/角加速度上限在近期数据中没有饱和，暂无继续增大证据。

### 4.4 Costmap

```yaml
global_costmap:
  robot_radius: 0.05
  inflation_radius: 0.30  # 当前已应用的暂停值
  cost_scaling_factor: 7.5
local_costmap:
  footprint: [[0.08,-0.08], [0.08,0.08], [-0.08,0.08], [-0.08,-0.08]]
  footprint_padding: 0.01
  inflation_radius: 0.18
  cost_scaling_factor: 15.0
```

GlobalPlanner 使用小 `robot_radius` 保留全局可达性，avoidance TEB 的 `0.30 m` 方形 footprint
负责局部保守检查。因此 selector 和 avoidance TEB 必须拦住对大几何过窄的全局路径。

### 4.5 Rolling selector / handoff

```yaml
channel_max_waypoints: 10
channel_waypoint_position_tolerance: 0.18
channel_waypoint_yaw_tolerance_deg: 90.0
channel_fan_radii: [0.45, 0.70, 0.95, 1.05]
channel_fan_half_angle_deg: 60.0
channel_fan_angle_step_deg: 10.0
channel_min_forward_progress: 0.20
channel_entry_depth_floor: 0.20
channel_min_entry_depth_progress: 0.20
channel_entry_depth_backtrack_tolerance: 0.05
channel_trigger_lookahead: 1.00
channel_horizon: 0.55
channel_trigger_clearance: 0.36
channel_hard_min_clearance: 0.20
channel_desired_clearance: 0.36
channel_handoff_min_waypoints: 2
channel_handoff_min_entry_depth: 0.90
channel_handoff_clearance: 0.28
channel_handoff_position_tolerance: 0.15
channel_waypoint_timeout: 45.0
channel_handoff_timeout: 88.0
channel_rollback_timeout: 45.0
channel_max_reselections: 2
channel_failed_candidate_exclusion_radius: 0.30
channel_reselection_escape_min_clearance: 0.16
channel_make_plan_tolerance: 0.0
```

中间点期望朝向是“当前机器人→选中点”的来向切线。`90°` 比原始 `180°` 更能抑制横着穿过锥桶，
又避免 `60°` 曾导致的方形 footprint 在窄口内持续调姿超时。

## 5. 前序路线：必须保留的用户修改

路线源为
[`pickup_staging_dev.yaml`](../src/smart_factory_mission/config/pickup_staging_dev.yaml)，其中包含用户手动调整和注释点。
不要“恢复所有注释点”，也不要从 main 分支整体覆盖。当前关键原始 seq：

- seq16: `(2.5915396308898926, -0.09775260388851166, -1.4098430871963501)`；
- seq17: `(2.61, -0.2764671516418457, -1.5845879316329956)`；
- seq18: `(2.605, -0.5809606552124023, -1.6079150438308716)`；
- seq21: `(2.291951103210449, -0.995, -3.104994773864746)`；
- seq22: `(0.7525417423248291, -0.995, 2.759993314743042)`。

实际执行是 30 个拟合目标的逐点导航，不是一次性 Path。当前约束：

- 必须映射到执行点的原始 seq：`[16, 17, 18, 21, 22]`；
- 必须同时满足位置和朝向：`[17, 18, 21]`；
- seq22 只按位置半径通过，不要求朝向；
- 中间点通过半径 `0.15 m`，特殊 seq 朝向容差 `0.15 rad`；
- 最终 seq35：位置 `0.15 m` 且朝向 `0.04 rad`。

修改路线源后必须重新生成
[`pickup_staging_fitted_path.yaml`](../src/smart_factory_navigation/config/pickup_staging_fitted_path.yaml)
和渲染图，并核对 30 个执行点、必需 seq 映射和地图占用检查。

## 6. 最近三组关键证据

### stage38 v1：作废

运行期间用户临时修改了 seq12，随后要求重启。本轮不计入统计。

### stage38 v2：零碰撞成功，但耗时较长

- 参数：中间点朝向 `60°`，全局膨胀 `0.33 m`；
- 结果：任务成功、零碰撞，`353.5 s`；
- 前 3 个滚动点后 handoff；HCP-TEB 超时后回退上一确认点并重选，随后完赛；
- 证明“超时→回退→排除失败候选→重选”主链有效。

日志：
[`stage38 v2 summary`](../script/logs/fixed_cone_e2e_stress_20260815_005310_adaptive_teb_stage38_rollback_escape0p16_yawtol60_v2/summary.json)

### stage38 v3：零碰撞安全失败

- 参数：中间点朝向 `60°`，全局膨胀 `0.33 m`；
- 结果：无碰撞，但送货导航失败，`198.8 s`；
- 第一点 `(-1.393,-1.902)` 在 `45 s` 内未同时满足位置与朝向；
- 超时时已进入参考净空 `0.005 m` 的窄口，排除失败点后最佳脱困路径只能恢复到 `0.115 m`，
  低于安全下限，因此 fail-closed。

日志：
[`stage38 v3 summary`](../script/logs/fixed_cone_e2e_stress_20260815_005948_adaptive_teb_stage38_rollback_escape0p16_yawtol60_v3/summary.json)

### stage39：当前暂停配置，完赛但撞 cone_18

- 参数：中间点朝向 `90°`，全局膨胀 `0.30 m`；
- 结果：任务成功，`193.2 s`，但碰撞 `cone_18`；
- 动态点：`(-1.659,-1.949)`、`(-0.627,-2.345)`、`(0.252,-2.573)`；
- 3 点后正常 handoff，没有超时；
- `cone_18` 移动 `0.090 m`，首次移动在第一滚动 goal 开始约 `3.6 s` 后；
- 入口通过性改善，但安全裕量不足。

日志：
[`stage39 summary`](../script/logs/fixed_cone_e2e_stress_20260815_010630_adaptive_teb_stage39_yawtol90_globalinfl0p30_v1/summary.json)

录像/世界状态：
[`stage39 evidence`](../data/cone_zone/end_to_end_stress/fixed_cone_e2e_stress_20260815_010630_adaptive_teb_stage39_yawtol90_globalinfl0p30_v1/)

## 7. 恢复后的调参顺序

stage38 v3 同时使用 `60° + 0.33 m`，stage39 同时使用 `90° + 0.30 m`，现有结果不能把改善或碰撞
单独归因于某一个参数。恢复时执行单变量实验：

1. 保留 `channel_waypoint_yaw_tolerance_deg=90.0`，只将全局 `inflation_radius` 从当前 `0.30`
   恢复为 `0.33 m`。
2. 如果 `90° + 0.33 m` 零碰撞完赛，保留该组合并开始 source009 连续 `3/3`。
3. 如果该组合仍无法进入，再只将全局膨胀改为 `0.32 m`。
4. 如果仍撞 `cone_18`，优先分析第一滚动点路径净空和车身朝向，考虑只提高第一点最低路径净空门槛。
5. 不通过缩小 avoidance footprint、`min_obstacle_dist` 或横移能力换取入口可达性。
6. source009 连续 `3/3` 前，不扩展到 007/012/032 的新一轮调参。

下一轮不要同时修改膨胀、朝向容差、footprint 和扇形评分，否则无法归因。

## 8. 运行与监控

将全局膨胀改为 `0.33` 并完成测试后，下一轮建议指令：

```bash
cd /home/ianichinose/gazebo_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 script/run_fixed_cone_e2e_stress_trials.py \
  --rounds 1 \
  --source-round 009 \
  --gui \
  --experiment-label adaptive_teb_stage40_yawtol90_globalinfl0p33_v1
```

不要省略 `--rounds 1`，否则会使用脚本默认轮数。脚本自动启动仿真、恢复锥桶、等待依赖、启动录像与碰撞/诊断监控并发布任务。

运行中查看详细日志：

```bash
tail -f script/logs/<run>/round_001/roslaunch.log
rg 'delivery channel laser waypoint|dynamic-selector handoff|timed out|rolling back|forced=True|TASK_FAILED|success=' \
  script/logs/<run>/round_001/roslaunch.log
```

检查残留进程：

```bash
pgrep -af 'gazebo|gzserver|gzclient|rviz|roslaunch|roscore|run_fixed_cone_e2e_stress_trials|send_navigation_task'
```

不要同时启动两个 Gazebo。历史上双 Gazebo 会造成仿真时钟停滞、控制环超期和脚本导航卡死。

录像删除先预览：

```bash
python3 script/prune_successful_gazebo_recordings.py --run <run-directory-name>
```

确认只包含完全合格成功轮后再删除：

```bash
python3 script/prune_successful_gazebo_recordings.py --run <run-directory-name> --apply
```

stage38 v3 和 stage39 是失败/碰撞证据，应该保留。不确定时不要添加 `--apply`。

## 9. 构建与回归

```bash
cd /home/ianichinose/gazebo_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

python3 -m py_compile \
  src/smart_factory_navigation/src/smart_factory_navigation/client.py \
  src/smart_factory_mission/src/smart_factory_mission/mission_server.py \
  src/smart_factory_mission/src/smart_factory_mission/delivery_entry_selector.py

python3 -m unittest discover -s src/smart_factory_mission/test -p 'test_*.py'
python3 -m unittest discover -s src/smart_factory_navigation/test -p 'test_*.py'

export PYTHONPATH="$PWD/script:${PYTHONPATH}"
python3 -m unittest \
  script/test_run_fixed_cone_e2e_stress_trials.py \
  script/test_prune_successful_gazebo_recordings.py \
  script/test_record_adaptive_teb_diagnostics.py

git diff --check
```

最后一次完整回归：mission 53 项、navigation 55 项、脚本 56 项，共 164 项通过。
最新 `90°` 和 `0.30 m` YAML 修改后又重跑 mission 53 项及 `git diff --check`，均通过。

## 10. 核心文件索引

| 文件 | 作用 |
|---|---|
| [`simu_requirement.md`](simu_requirement.md) | 最新合规要求 |
| [`锥桶区导航纪要.md`](锥桶区导航纪要.md) | 完整历史调参与仿真记录 |
| [`保险设计需求纪要.md`](保险设计需求纪要.md) | 各阶段保险/fallback 需求 |
| [`adaptive_teb_params.yaml`](../src/gazebo_nav/launch/config/move_base/adaptive_teb_params.yaml) | selector、baseline、avoidance 参数 |
| [`costmap_common_params.yaml`](../src/gazebo_nav/launch/config/move_base/costmap_common_params.yaml) | 全局代价地图膨胀 |
| [`local_costmap_params.yaml`](../src/gazebo_nav/launch/config/move_base/local_costmap_params.yaml) | 局部代价地图几何和膨胀 |
| [`smart_factory_adaptive_teb/`](../src/smart_factory_adaptive_teb/) | 自适应局部规划器和激光 selector |
| [`delivery_goals.yaml`](../src/smart_factory_mission/config/delivery_goals.yaml) | 准备位姿、动态通道、handoff 参数 |
| [`delivery_entry_selector.py`](../src/smart_factory_mission/src/smart_factory_mission/delivery_entry_selector.py) | 扇形候选、净空评估、排除和重选 |
| [`mission_server.py`](../src/smart_factory_mission/src/smart_factory_mission/mission_server.py) | 动态 goal 循环、avoidance lock、回退 |
| [`client.py`](../src/smart_factory_navigation/src/smart_factory_navigation/client.py) | 单次导航墙钟超时与 action 取消 |
| [`route_executor.py`](../src/smart_factory_navigation/src/smart_factory_navigation/route_executor.py) | 30 点逐点导航及特殊 seq 判定 |
| [`run_fixed_cone_e2e_stress_trials.py`](../script/run_fixed_cone_e2e_stress_trials.py) | 固定锥桶端到端压力测试 |
| [`record_adaptive_teb_diagnostics.py`](../script/record_adaptive_teb_diagnostics.py) | 自适应模式与不可行证据 |
| [`prune_successful_gazebo_recordings.py`](../script/prune_successful_gazebo_recordings.py) | 合格录像清理 |

## 11. 工作树注意事项

`TEB_test` 在 `ddcb359` 之后包含大量尚未提交的实现、配置、用户路线修改和试验证据。

- 不要使用 `git reset --hard`、`git checkout -- <file>` 等方式清理工作树；
- 不要从 main 整体覆盖 move_base、路线、mission 或 navigation 配置；
- 不要删除未跟踪的失败/碰撞录像，它们是调参归因证据；
- 每次仿真前确认没有残留 Gazebo/ROS；
- 每次只修改一组可归因参数，并在 `experiment-label` 中写明参数；
- 任一轮出现碰撞、卡死、导航失败或识别失败，连续计数归零。

## 12. 最短恢复清单

1. 阅读本文、最新规则和《锥桶区导航纪要》的 2026-08-15 暂停节。
2. 确认分支 `TEB_test`，保留脏工作树和用户路线。
3. 确认没有残留 Gazebo/ROS。
4. 保留中间点朝向 `90°`，只将全局膨胀 `0.30→0.33 m`。
5. 运行测试和 `git diff --check`。
6. GUI 运行 source009 单轮，重点观察第一滚动点和 `cone_18`。
7. 以任务结果、碰撞监控、selector 日志三方证据判定。
8. 首轮零碰撞完赛后，才继续 source009 连续 `3/3`。
