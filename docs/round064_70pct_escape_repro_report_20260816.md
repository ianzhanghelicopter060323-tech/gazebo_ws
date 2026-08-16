# round-64 卡死位姿复现与 70% 逃逸距离压力测试报告

日期：2026-08-16
分支：`TEB_test`
涉及运行：`pre_cone_e2e_trials_20260816_023051_seed10786202215795395888 / round_064`

## 一、结论摘要

1. **70% 前/后净空动态逃逸距离已实现并通过单元测试**（代码、配置、测试见第二节）。
2. **按记录位姿静态放置底盘做的一轮仿真压力测试未能复现卡死**：前方净空实测 0.581 m（而非第 64 轮记录的 0.056 m），对 seq35 补充观察位姿的 `NAVIGATE_POSE` 在 63.8 s 内成功到达，未触发任何逃逸（`escape_events=[]`）。
3. 根因：第 64 轮的卡死是**动态物理骑卡**（`gazebo_ros_planar_move` 底盘在行进中骑上障碍，pitch≈67°、离地 14 cm、冻结约 35 s），**不是**某个可静态摆放的位姿。记录的 0.056 m 前方净空是**激光打地面造成的假象**，不是真实障碍。
4. 因此这一轮测试只证明了"从记录位姿出发、导航可成功"，**没有真正压测到 70% 逃逸逻辑**。若要压测逃逸，必须改用动态回放或确定性卡死注入（见第五节）。
5. 报告的问题（P1–P6）聚焦：**姿态/倾斜安全门缺失（最高优先）**、倾斜+单侧激光时选向不可靠、静态复现方法学不适用、物理卡死无法靠距离缩放解决。

## 二、70% 逃逸距离：实现与验证

| 项 | 位置 | 内容 |
| --- | --- | --- |
| 参数 | `src/smart_factory_navigation/config/navigation.yaml` | `recovery.distance_scale_factor: 0.7` |
| 计算 | `src/smart_factory_navigation/.../base_alignment_controller.py` | `_escape_max_distance_for(clearance) = min(0.7 * clearance, max_distance=0.08)`；净空未知时回退到 `0.08` |
| 选距 | 同文件 `escape()` | 选定方向（前/后）后，用该方向净空计算动态目标距离；循环以 `moved >= dynamic_max_distance` 提前结束；`logwarn` 增加 `scale=… target=…m` |
| 测试 | `test/test_base_alignment_controller.py` | 新增 `test_escape_max_distance_scales_with_clearance`：`0.056→0.0392`、`0.2→0.08`、`nan→0.08`、`1.0→0.08`，全部通过 |

行为语义（按用户决定保持不变的部分）：方向选择仍是"前后都有效取较大、仅前有效则无条件前进"——**不加入安全距离门控**，因为窄通道会永远无法触发逃逸。

## 三、复现方法、结果与证据链

### 3.1 方法

- 场景数据：`script/config/round064_escape_repro.json`。底盘**平放**在记录位姿 `(-1.39, -0.486, yaw=-0.587)`；cube/cone 使用正确 run（`20260816_023051`）`cube_scene.json` 的初始 spawn 位姿。
- 目标：seq35 补充观察位姿 `(-1.2243, -0.525, yaw≈0)`——正是第 64 轮失败的那次 `navigating to requested pose`。
- 执行：`script/reproduce_round064_escape.py`：headless 起仿真 → 放置场景 → `/initialpose` 重置 amcl → 静置 12 s → 读 `/scan` 前/后净空 → 发送 `NAVIGATE_POSE` → 等结果 → 从 `roslaunch.log` 收割逃逸事件。
- 产物：`script/logs/round064_escape_run_20260816_172918/`（`result.json`、`scene_setup.log`、`roslaunch.log`）。

### 3.2 结果

```
reproduced clearance front=0.581 rear=nan
feedback stage=5 ... navigating to requested pose (attempt 1/2)
navigation result state=3 success=True error_code=0 message=requested pose reached
result: front_clearance=0.581, rear_clearance=NaN, goal_state=3(SUCCEEDED),
        duration=63.8s, escape_events=[]
```

场景放置校验通过（三个 cube 误差 < 1e-9 m）。平放位姿下前方是 0.581 m 的开阔空间，导航自由到达目标。

### 3.3 为什么静态复现失败（证据链）

1. **世界录制**（`gazebo_world_state.log`，sim 87–204 s）：卡死窗口 `car3 pitch=1.18 rad(67°), z=0.149`（静止时 z≈0.009）；cube 与 cone 全程未移动，始终在初始位姿。
2. **0.056 m = 激光打地假象**。激光装在 `laser_link`（`z≈0.0874 m`、前伸 `0.146 m`）。pitch 67° 时前束俯角 67°，打地距离 = 激光离地高度 / tan(67°) ≈ **0.02–0.056 m**，与记录吻合；pitch 时后束指向天空 → `rear_clearance=unavailable`。第二次恢复（pitch≈0.5 rad）记录的 `front=0.205` 同样落在打地距离 0.165–0.205 m 区间。**两次"前方净空"读数都只是 pitch 的函数，不是障碍物。**
3. **静态世界没有物理碰撞**。`math.world` 中墙面、`math` 道具、`area_*` 均为视觉-only；唯一有碰撞的是 cube（0.04 m³、0.02 kg）和 cone。物理上不存在能挡住/抬起底盘的固定结构。
4. **驱动是 `gazebo_ros_planar_move`**（`car3.urdf`）：只对 `base_link` 施加 XY 平面力 + 绕 Z 角速度，**不管理 pitch/roll**；20 kg COM 在 z≈0.035 m。撞上小障碍物时推力在接触点上方形成力矩，底盘天然翘头、骑上障碍并保持。
5. **骑卡是过程而非位姿**：俯仰在录制空窗（117–162 s）内已到 0.535 rad，162 s 后仍能通过 waypoint 27/28/29，到 waypoint 30 才卡住（触发恢复 #2），随后 pitch 升至 1.18 并在 seq35 补充导航期间冻结约 35 s（恢复 #3）。记录的"卡死位姿"只是该过程末尾的一帧。

## 四、发现的问题（可能的问题清单）

### P1 姿态/倾斜安全门缺失 —— 最高优先级
倾斜会**整体污染净空输入**：前束打地给出虚假的小净空，后束给 `NaN`。第 64 轮 `front=0.056` 就是这一假象，却被当作"前方 5.6 cm 障碍"用于选向。handoff 建议的 `RECOVERY_UNSAFE_TILT` 姿态安全门**不是可选项而是必需**：仿真中读 `gazebo/model_states` 的 roll/pitch，实机读 IMU；逃逸前与速度循环内检查，超阈即零速停机并输出明确安全错误。

### P2 倾斜 + 单侧激光时"无条件前进"仍会把车推向卡点
按用户决定保留了"仅前向有效即前进"（不做安全距离门控）。但第 64 轮证明该分支在输入被倾斜污染时是危险路径：`rear=NaN, front=0.056`（假象）→ 前进 → 把骑卡的车继续往前顶 8 mm。建议：检测到显著 roll/pitch 时禁止前进分支、改走姿态安全门；净空逻辑本身保持不加安全门。

### P3 静态摆放无法复现动态卡死 → 压力测试方法学问题
"卡死位姿"是动态过程的某一帧，不是可静态摆放的状态。现有固定场景执行器 `run_pre_cone_failure_scenarios.py` 与本复现脚本**都无法真正压到逃逸链路**。要压测逃逸，必须使用：(a) 同种子完整 mission 回放（录制骑卡起点），或 (b) **确定性卡死注入**——在路径上放置真实碰撞障碍、底盘水平紧贴其边缘，让净空读数是真实的障碍距离。

### P4 物理卡死无法靠"距离缩放"解决
即使目标距离从固定 0.08 m 缩到 `0.7×0.056=0.039 m`，被骑卡的车 2 s 只移动 8 mm，仍会判定失败（错误码 7）。70% 逻辑在**真实窄缝隙**场景是合理的安全改进（避免 0.08 m 过冲撞上对面），但对骑卡类卡死无效。需要不同机制：姿态安全门停机 / 反向+转向 / 提高脱困力 / 关节或轮滑诊断。

### P5 无进展/位移判据对"骑卡打滑"不敏感
现有 `escape()` 用位移（≥20 mm 视为有效）判成败，8 mm 位移直接判失败并升级。缺少"轮滑/悬空"诊断，无法区分"真前方障碍"与"假前向读数+物理卡死"。建议记录 wheel slip / 底盘姿态，把 P1 的姿态事件纳入逃逸结果结构体。

### P6 日志可观测性不足
录制空窗 117–162 s 掩盖了骑卡起点；无接触数据（`/gazebo/contacts`）、无 roll/pitch 时间序列事件。后续压力运行应：开 `/gazebo/contacts` 或提高 `model_state` 采样；落实 handoff 第 3 条的 `NO_PROGRESS_DETECTED / RECOVERY_START / RECOVERY_DONE` 结构化事件（含 roll/pitch）。

## 五、下一步建议

1. **实现姿态安全门**（P1，最高优先），并用合成"倾斜底盘"用例做单元/集成验证。
2. **改压力测试输入**：用确定性卡死注入压测 70% 逃逸，或同种子全 mission 回放录制骑卡起点；`round_064_recovery_tilt` 场景应定义为"动态注入"而非"静态摆放"。
3. **更新 handoff 文档**：注明第 64 轮静态复现不可行、`0.056 m` 为倾斜假象；`round_064_recovery_tilt` 的验收标准改为"能检测到倾斜安全门并安全终止"。
4. 保留本次运行产物 `script/logs/round064_escape_run_20260816_172918/` 作为"静态复现不可行"的实证。

## 六、测试产物清单

- 复现运行：`script/logs/round064_escape_run_20260816_172918/`（`result.json`、`scene_setup.log`、`roslaunch.log`、`case.json`）
- 复现脚本：`script/reproduce_round064_escape.py`
- 场景数据：`script/config/round064_escape_repro.json`
- 单元测试：`src/smart_factory_navigation/test/test_base_alignment_controller.py::test_escape_max_distance_scales_with_clearance`
- 相关日志证据：`script/logs/pre_cone_e2e_trials_20260816_023051_seed10786202215795395888/round_064/roslaunch.log`、`data/teb_pre_cone/pre_cone_e2e_trials_20260816_023051_seed10786202215795395888/round_064/gazebo_world_state.log`
