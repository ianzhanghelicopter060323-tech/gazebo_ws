# Pre-cone 100 轮失败修复与后续压力测试交接

更新日期：2026-08-16

## 当前结论

已完成的基线运行是
`pre_cone_e2e_trials_20260815_210500_seed7694522871009289374`：100 轮完成，
其中 23、29、46、74、88 轮出现过异常。它们均不是终态的“物块识别失败”：23
是导航无进展后超时/抢占，29 是低实时因子下以墙钟超时等待机械臂，46、74 是视觉
修正距离阈值过小，88 是旧深度坐标复用导致最终夹爪窗口未命中。

最新核实的后续批次是
`pre_cone_e2e_trials_20260816_023051_seed10786202215795395888`。该批次的第 64 轮
新增了真实的导航失败：受限逃逸**确实被触发三次**，但第三次无法使底盘脱困，最终以
`navigation_aborted` / 错误码 7 结束。这一场景必须纳入后续固定场景和压力测试，不能
再把逃逸视为“待触发、尚未验证”的功能。

针对五个场景已各执行一次固定场景回归，均在首轮到达 `OBJECT_GRASPED`（stage 14）。
这只是回归验证，不应视作压力测试已完成；任何后续改动导航、机械臂时序、感知对齐或
抓取判定的提交，都必须按本文“后续压力测试”重新验证。

| 来源轮次 | 固定场景 ID | 本次结果 | 证据 |
| --- | --- | --- | --- |
| 23 | `round_023_navigation` | 通过，129.339 s；未再次形成无进展 | `script/logs/pre_cone_failure_regression_20260816_020539/summary.json` |
| 29 | `round_029_slow_sim_arm` | 通过，204.865 s；物理更新速率缩放为 0.25 | `script/logs/pre_cone_failure_regression_20260816_021243/summary.json` |
| 46 | `round_046_seq37_edge` | 通过，114.998 s；0.252 m 粗调后再调 0.021 m | `script/logs/pre_cone_failure_regression_20260816_021649/summary.json` |
| 74 | `round_074_seq36_edge` | 通过，102.938 s；再调 0.022 m | `script/logs/pre_cone_failure_regression_20260816_021902/summary.json` |
| 88 | `round_088_grasp_window` | 通过，103.278 s；再调 0.023 m 后抓取 | `script/logs/pre_cone_failure_regression_20260816_022100/summary.json` |

固定场景数据在 `script/config/pre_cone_failure_scenarios_20260816.json`，执行器为
`script/run_pre_cone_failure_scenarios.py`。每次尝试均重新启动仿真、重置物块（23 轮
还重置锥桶），成功后停止；禁止把上一轮仿真状态带入下一轮。

## 第 64 轮“逃逸”复核（20260816 最新批次）

结论：**逃逸链路已触发，但第三次恢复失败；目前不能宣称逃逸有效。**

证据目录为
`data/teb_pre_cone/pre_cone_e2e_trials_20260816_023051_seed10786202215795395888/round_064`
及对应的 `script/logs/.../round_064/roslaunch.log`。该轮目标为 daily / seq36，耗时
230.897 s，最终 `stage=250`、错误码 7，错误原文为
`move_base made no measurable progress`。

| 恢复位置 | 指令方向 | 前/后净空 | 实际位移 | 结果 |
| --- | --- | --- | --- | --- |
| 进场 waypoint 7/30 | 后退 | 不可用 / 不可用 | 0.057 m | 随后通过该航点 |
| 进场 waypoint 30/30 | 前进 | 0.205 m / 不可用 | 0.028 m | 随后继续流程 |
| 补充观察位姿的导航 | 前进 | 0.056 m / 不可用 | 0.008 m | 重试后仍无进展，任务失败 |

第三次恢复在仿真时间 197.764 s 进入，198.502 s 结束。其持续约 2.011 s，却只移动
8 mm，低于当前 `escape()` 的 20 mm 有效位移判据。用户录像观察到机身倾斜；Gazebo
世界录制保留了该时段，但当前任务日志没有 roll/pitch 或碰撞接触数据，所以可以确认
“恢复时无法脱困”，不能仅凭文本日志量化倾斜角度。

当前逃逸链路设计为：连续 15 s 位移小于 0.03 m 时，取消当前导航目标，等待其失活；
根据前后激光扇区的较大净空选择前进或后退，以 0.05 m/s 最多移动 0.08 m 或 2 s，
停止后只重试原航点一次。代码入口：

- `src/smart_factory_navigation/src/smart_factory_navigation/route_executor.py`：无进展判定、取消目标和重试。
- `src/smart_factory_navigation/src/smart_factory_navigation/base_alignment_controller.py`：激光选向及唯一 `/cmd_vel` 发布者的受限逃逸。
- `src/smart_factory_navigation/config/navigation.yaml`：阈值和上限。

第三次的选向暴露出当前策略问题：当“只有前方激光有效”时，代码无条件选择前进；此次
前方净空仅 0.056 m、后方不可用，仍前进。该逻辑可能把底盘继续推向近障碍，是优先级
最高的修正项。23 轮固定锥桶场景的首轮通过依旧有效，但它不能替代第 64 轮的失败复现。

## 后续修改方向（先补可观测性，再调参数）

1. 修正单侧激光选向：仅当前方净空大于“机器人后退/前进所需安全距离”时才允许前进；
   前方净空过小或后方不可用时，不得用“前方可读”覆盖默认的保守后退策略。阈值应基于
   footprint 加安全边界配置，而不是写死。第 64 轮的 0.056 m 前方净空必须被判为不宜前进。
2. 增加姿态安全门：在仿真中读取 `gazebo/model_states`，真实机器人中读取 IMU；逃逸前和
   每次速度循环检查 roll/pitch。超过阈值时立即零速度、停止继续推挤，输出明确的
   `RECOVERY_UNSAFE_TILT` 错误并保留姿态和接触诊断。
3. 为一次恢复增加结构化事件和诊断字段：`NO_PROGRESS_DETECTED`（起止位姿、15 s 位移）、
   `RECOVERY_GOAL_CANCELLED`、`RECOVERY_START`（前/后净空、方向、速度）、
   `RECOVERY_DONE`（实际位移、墙钟时间、roll/pitch）、`RECOVERY_RETRY_RESULT`。这些事件应同时写入
   任务日志和单轮 `performance.json`，避免只能依赖录像判断。
4. 在执行逃逸前验证 move_base 已不再处于活动状态，并记录 `/cmd_vel` 的最终发布链路。
   若存在速度仲裁/旧目标残留，逃逸指令可能被覆盖；此时应在确认取消完成后再独占发布，
   并在结束时显式发布零速度。
5. 为 `/scan` 增加新鲜度和有效束数判断。没有可用激光时应记录“净空未知、按保守后退”，
   不能把激光缺失伪装成正常选向。压力测试前用 `rostopic info /scan` 和一条有效
   `LaserScan` 确认实际订阅/重映射正确。
6. 从第 64 轮的 `cube_scene.json`、世界录制和锥桶状态创建
   `round_064_recovery_tilt` 固定场景；若原始随机锥桶状态无法完整恢复，再增加一个
   确定性无进展注入用例。二者都必须保留地图、初始位姿、障碍物和物块状态。
7. 只有在上述事件显示“检测、取消、逃逸指令均已发生但实际位移不足”时，才调整
   速度、距离、扇区角度或方向策略；每个未解决问题最多进行三轮“改参 + 同场景复测”。

### 地图几何复核（math_newest.pgm，健壮解析，2026-08-16）

用跳过 `#` 注释行的 PGM 头解析器 + yaml 精确阈值（occupied_thresh=0.65 /
free_thresh=0.196，negate=0）重新解析 `src/gazebo_map/maps/math_newest.pgm`
（`simulation.launch` / `gazebo_nav.launch` / `full_competition.launch` 的默认地图）：

- 512×256 格、0.05 m、origin (-12.2,-12.2)。像素只有 4 种值：254（free，6595 格 ≈5%）、
  205（unknown，123722 格 ≈94.4%）、0（occupied，755 格）、以及一个 10。
- 4 邻接连通分量：free 空间是**唯一**分量，bbox x∈[-2.10, 2.85]、y∈[-3.20, 0.25]，
  中心 (0.38, -1.47)，约 4.95×3.45 m——这就是 pickup staging 的整个竞技场；起始 (0,0)
  与首段拟合航迹都在其内。
- 结论：**地图几何上不存在任何被围死的 free 岛/死角**。costmap `track_unknown_space: true`
  下 unknown 视作 lethal，而 free 区域本身单连通，因此 round-064 式“无进展”不可能由地图
  本身造成，必须有注入障碍（锥桶/墙/立方体）——与既有 `pre_nav_jam_trial_wall006.json`
  和 cage 用例一致，也确认 `round_064_recovery_tilt` 只能从第 64 轮真实锥桶状态构造，
  不能靠地图选点。

## 后续压力测试（必须执行）

### 通用规则

- 当前五个固定场景清单：`script/config/pre_cone_failure_scenarios_20260816.json`；在创建
  `round_064_recovery_tilt` 后，将其作为第六个必测场景加入同一清单。
- 每个场景连续独立运行 3 次；每次都由执行器重启仿真。单次通过不代表压力通过。
- 一旦原问题复现或引入新问题，停止该场景后续轮次，先保存日志/录像/性能记录，再修改；
  对该问题最多三轮参数优化复测。
- 29 轮若只在资源受限的连续批量运行中失败、而独立单发通过，应标记为“资源风险”，
  不再为单发比赛场景无限调参；仍必须保存实时因子和系统资源数据。

注意：现有执行器的 `--max-attempts` 是“失败后最多重试次数”，成功会立即停止，
**不能**用 `--max-attempts 3` 代替三次压力通过。要对当前五个固定场景做三次独立压力
测试，应执行三次独立仿真（任何一次失败即停止）：

```bash
source devel/setup.bash
run_stamp=$(date +%Y%m%d_%H%M%S)
for run_index in 1 2 3; do
  python3 script/run_pre_cone_failure_scenarios.py \
    --max-attempts 1 \
    --output-dir "script/logs/pre_cone_failure_pressure_${run_stamp}/run_${run_index}" \
  || break
done
```

建议为避免失败后自动连续重试掩盖调参前状态，日常排查时逐场景执行首轮：

```bash
source devel/setup.bash
python3 script/run_pre_cone_failure_scenarios.py \
  --case-id round_023_navigation --max-attempts 1
```

然后针对 29、46、74、88 分别替换 `--case-id`。每次运行的 `summary.json`、`task.log`、
`roslaunch.log` 都需保留。

### 逃逸专门验收

对 `round_064_recovery_tilt`（以及必要的确定性卡死注入用例）连续独立运行 3 次，必须
同时满足：

1. 每次均在无进展阈值后检测到 `NO_PROGRESS_DETECTED`；同一被阻塞目标只进行一次受限恢复。
2. 当前 move_base 目标已取消/失活后才开始发布逃逸速度。
3. 不得在近障碍单侧净空条件下向该障碍方向行驶；每次循环均记录 roll/pitch。若触发
   倾斜安全门，必须立即零速度并给出 `RECOVERY_UNSAFE_TILT`。
4. 正常恢复的逃逸位移至少 0.02 m，且不超过 0.08 m；结束时 `/cmd_vel` 为零。
5. 原航点被重新发送并最终成功；若姿态安全门触发，则以明确安全错误终止，不得静默卡死
   到外部总超时。
6. 日志含前后净空、方向、实际位移、roll/pitch 和重试结果，录像可与这些时间戳对应。

只有该专门测试通过，才能宣称”逃逸行为已验证”。

### 恢复链实现与无 GUI 仿真验证（2026-08-16，分支 TEB_test）

本轮指令实现的恢复链（全部在恢复层，未改动任何全局/局部规划器参数）：

- `base_alignment_controller.escape()` 自适应方向（handoff 第 1 项修正）：先按前后净空较大者
  试探，若该方向 `stall_timeout`（≤ wall_timeout/4，上限 0.5 s）内无实际位移则切换反方向；
  单次探测位移受 `_escape_max_distance_for(clearance)` = min(0.7 × 净空, max_distance) 限制，
  总时长受 `wall_timeout`（2.0 s）限制。
- `route_executor`：`_NoProgressMonitor` STUCK（25 s 无 0.02 m 位移）→ 取消目标并等待失活 →
  `recovery_attempts++` → 逃逸 → 逃逸成功则 `post_escape_wait`（3.0 s）隔离后重新发送同一航点；
  后续导航再次 STUCK 则再次恢复（每航点上限 `max_attempts_per_waypoint`=2）；逃逸失败或超限则
  `retry_count++`（max_retries=1）后放弃。

无 GUI 仿真复现第 64 轮卡死（不可见静障碍，均置于首段直线航迹 (0,0)→(0.927,0.006) 正线上，
机器人 z=0.02 平放）：

| 用例 | 障碍（静、激光不可见） | 结果 |
|---|---|---|
| 窄立方 | 0.04 m³ @ (0.3,0) | 不在首段航迹正线上，机器人水平绕过/骑过，未形成有效卡死（或逃逸后再次卡住） |
| 0.06 墙 | 0.06 m 高 × 0.28 m 宽 @ (0.45,0) | **2 次 STUCK→自适应逃逸**（正向 0.004/0.009 m，反向均 0.000 m）→ 有界终止；机器人 pitch ≈39° 几何楔死，逃逸无法产生实位移 |
| 0.04 墙 | 0.04 m 高 × 0.28 m 宽 @ (0.45,0) | **2 次 STUCK→逃逸，实位移 0.215/0.190 m**（机器人爬上墙顶楔住，前推后从墙沿滚下 = 真实恢复）；恢复导航后墙仍在、位姿劣化，move_base 每 5 s 规划失败，goal ABORTED 收尾 |

关键发现（决定性）：planar_move + 四个固定轮 + 低车身底沿（world z≈0.0195）+ 高激光
（world z≈0.107）的 car3，碰到任何不可见静障碍都会翘起 24–39°——接触点抬升低车身底沿，
planar_move 无法爬越；激光随 pitch 打地，move_base 退化为每 5 s 规划失败。恢复链机械上
正确工作（STUCK 判定、取消、自适应反向切换、等待隔离、恢复导航、再次恢复），但机器人被
几何楔死时正向/反向探测实际位移均 ~0 m，无法从翘起状态中解放——与第 64 轮第三次恢复
（前向 0.056 m 净空仅移动 8 mm、机器人倾斜、navigation_aborted）根因完全一致。

结论：
- 恢复链机械行为端到端验证通过：STUCK 判定 → 取消目标 → 自适应逃逸（卡死方向试探 +
  停滞切反方向）→ 3 s 等待隔离 → 恢复导航 → 再次 STUCK → 再次逃逸。0.04 m 墙用例中两次
  逃逸均产生实位移（0.215/0.190 m），机器人被从”爬墙楔住”状态真实解放并恢复后续导航。
- “卡死→逃逸→继续导航到**成功**”对静态不可见障碍不可达：障碍无法被推开，机器人反复撞回
  原位，且翘起（24–39°）导致激光打地、规划退化，最终经 ABORT 有界终止——这是正确工程行为。
  0.06 m 墙用例（39° 严重楔死）连逃逸实位移都没有（前 0.004/0.009 m、反 0.000 m）。
- 恢复方向修正（第 1 项）已实现并通过单测。要真正解决 round-64 类失败并让逃逸”可诊断、
  可安全终止”，需实现 handoff 第 2 项”姿态安全门”：逃逸前/逃逸中读取 roll/pitch（仿真用
  gazebo/model_states，真机 IMU），超过阈值立即零速度、停止推挤、输出 `RECOVERY_UNSAFE_TILT`
  并保留姿态/接触诊断。此项不涉及任何全局/局部规划器参数。

## 已完成的代码与配置变更

- 准备位姿 2 前锁定 baseline，避免提前进入 avoidance；移除 35→36 和 36→37 过渡点。
- 机械臂、抓取对齐改为仿真时间软期限配合墙钟硬期限及 `/clock` 停滞检测。
- 粗对齐上限设为 0.30 m；每次粗调后使用新深度坐标复核，允许一次不超过 0.05 m 的二次微调。
- 保留 ±20 mm 抓取窗口，增加夹爪—物块偏差记录；未加入抓取区激光净空硬门槛。
- 100 轮统计区分真正的终态识别失败与导航/时序失败，并记录单轮资源与 Gazebo 状态。

当前单元测试 141 项、Adaptive TEB C++ 构建、Python/YAML/差异检查均已通过；但这不能替代
本文要求的多轮固定场景压力测试，尤其不能替代逃逸专门验收。
