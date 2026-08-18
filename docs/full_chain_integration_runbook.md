# 智慧工厂整链运行手册

更新：2026-08-19

本手册只保留现场会用到的启动、观察、止损、取证和重开命令。规则以
`第二十一届讯飞智慧工厂竞赛规则总手册.md` 为准；全国总决赛再叠加总决赛手册。

## 1. 只有一条正式发车口令

导航、相机、任务栈和巡线节点都必须在计时前启动为待机。它们是**赛前准备命令**，
不是发车指令。

全国总决赛从现在起统一为：**一个 SSH 终端执行一次一键启动，脚本返回后仍在同一终端
执行状态检查；裁判开始计时后不再操作电脑，只说一次完整订单。** 不需要为导航、相机、
巡线、任务和语音分别保留终端。另开的 `rostopic echo` 仅用于实验室观察，不属于正式
发车流程，也不是车辆运行的必要条件。

裁判说“开始”后，本队唯一的正式发车动作是说完整订单：

```text
小飞小飞，前往物品领取区，取得<实体货品>，放置在对应仓库，并领取仿真环境中需要的<仿真货品>放置在对应仓库
```

例如：

```text
小飞小飞，前往物品领取区，取得香蕉，放置在对应仓库，并领取仿真环境中需要的数据线放置在对应仓库
```

只有在 `/mission/order` 尚未锁存、状态机仍为 `READY` 时才可重说；不能在裁判开始前说。

以前写的“决赛导航 launch 与基础导航 launch 二选一”只表示：赛前根据比赛阶段装载
对应底层栈，不能同时运行两套导航。它不表示比赛过程中有两条发车命令。

| 阶段 | 赛前底层导航 | 赛前巡线待机 |
|---|---|---|
| 初赛/省赛 | `ucar_nav2026 ucar_navigation2026.launch` | `visual_navigation_3 test.launch trace_edge:=0` |
| 全国总决赛 | `ucar_nav_finals ucar_navigation_finals.launch` | `visual_navigation_finals2 final2.launch trace_edge:=0` |

两阶段抽到直行都执行：模式 3 → 首岔口右转 → P1。

## 2. 不含真实仿真的测试流程（与正式流程隔离）

本节只用于测试实体任务链和任务 3 接口时序，使用 `visual-rehearsal` 的 5 秒仿真桩，
不连接 Gazebo，也不建立真实 TCP 仿真链路。它不能作为任务 3 正式验收证据，且不能与
第 6 节的 `competition` 正式流程同时运行。

### 2.1 当前边界

- 不含真实仿真的实体车整链使用 `visual-rehearsal`，任务三由明确标记的 5 秒仿真桩回执代替。
- 这可以检查任务 1、2、4、5 的实体链和任务 3 接口时序，但不能算正式仿真验收。
- `competition` 模式必须等真实 Gazebo 阶段 20 回执和现场点位全部验收后才能通过预检。
- 每次测试前必须清场、确认急停人员和 AMCL；不要从上一趟的中间状态直接重说口令。

### 2.2 初赛/省赛无仿真测试启动命令（一个终端）

先确认没有遗留的决赛巡线或导航节点；不能让两阶段底层栈同时运行。发现遗留时先使用
对应阶段的定向停止脚本，不能直接叠加启动。

实验室排练只执行：

```bash
cd ~/qhh
./scripts/mission/start_preliminary_field_stack.sh \
  --mode visual-rehearsal --arm-preliminary-field-stack
```

脚本依次启动初赛导航、ROS 相机、巡线待机、任务视觉和语音层。巡线节点提前在线，但在
收到合法方向码和 `88` 前不打开 `/dev/video0`，因此不会与任务 1-4 的 ROS 相机争用。
脚本不发布订单、方向、`88` 或速度。

上一条命令返回后，仍在同一终端执行检查：

```bash
./scripts/mission/status_preliminary_field_stack.sh visual-rehearsal
```

只有同时看到 `PRELIMINARY_SOFTWARE_READY`、`PRELIMINARY_FIELD_READY` 和 `CHAIN_READY`
后才继续人工确认：

1. 车辆在发车格并静止；
2. RViz 中 AMCL 与真实朝向一致；
3. `/scan`、`/odom`、`/camera/image` 连续；
4. `/cmd_vel` 发布者符合当前阶段；
5. 电量、赛道和急停人员就绪。

SSH 断线后不要重复冷启动，仍用状态脚本检查。整趟结束后定向停止：

```bash
cd ~/qhh
./scripts/mission/stop_preliminary_field_stack.sh visual-rehearsal
```

### 2.3 调试时可选观察终端

下面两个观察终端不是启动条件，正式比赛不需要打开：

状态机：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /mission/status
```

播报和任务五时限：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /mission/announcement_status
```

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /mission/task5_completion_status
```

任务五成功回执必须同时看到：

```text
state: COMPLETE
started_within_10_seconds: true
completed_within_30_seconds: true
```

### 2.4 每趟验收记录

| 项目 | 必须记录 |
|---|---|
| 订单 | 实体货品、仿真货品、`order_id` |
| 任务 1 | 三个 QR、两路 Spark 结果、播报文本与完成回执 |
| 任务 2 | 三厂牌结果、目标停车点、停稳、播报回执 |
| 任务 3 | 请求 ID、桩/真实回执、播报回执 |
| 任务 4 | 停止线状态、交通方向、交接模式、相机释放结果 |
| 任务 5 | `/visual_nav_end`、停稳、开始/完成时间布尔值 |
| 异常 | 首个异常状态、现场现象、快照目录 |

## 3. 卡死时怎么办

### 3.1 正式比赛

正式计时后任何队员不得触碰电脑、SSH、参数或人工发布控制消息。发生卡死时：

1. 先保证人员安全；
2. 立即向裁判报告现象；
3. 只有裁判明确授权后，发车员才能处置车辆；
4. 本轮按裁判口径结束或计分；
5. 利用两次挑战中的下一次重新冷启动，不在本轮中途改代码。

不要在正式比赛中使用 `rostopic pub /cmd_vel`、手工方向码、遥控或修改参数“救车”，
这些会破坏自主比赛证据，严重时取消本轮成绩。

### 3.2 实验室调试

车还在运动或运动趋势不明：先按物理急停/断底盘驱动，再取证。不要先 SSH 排查。

#### 口令说完后车辆没有启动

不要立刻重复口令、抱车或全量重启。车辆确认静止后，先在原 SSH 终端执行：

```bash
cd ~/qhh
./scripts/mission/status_finals_field_stack.sh visual-rehearsal
./scripts/mission/capture_factory_failure_snapshot.sh visual-rehearsal
```

初赛/省赛把第一条状态命令换成：

```bash
./scripts/mission/status_preliminary_field_stack.sh visual-rehearsal
```

- 看到 `FINALS_FIELD_BLOCKED` 或 `PRELIMINARY_FIELD_BLOCKED`：不得重说，保留完整输出和
  `SNAPSHOT_READY` 目录排查缺失节点、就绪门或状态机问题。
- 看到对应的 `*_FIELD_READY`、`CHAIN_READY`，并确认任务仍在 `WAIT_ORDER/READY`、没有
  `/mission/order` 锁存：才可清晰、匀速地重说一次完整订单。
- 唤醒词后停顿约 `0.5 s`，货品之间稍作停顿；不要拆成多条命令，也不要连续重复两遍。
- 若已出现订单 ID、`/mission/order` 或车辆开始响应，即使车辆暂时未动也不能重说，先按
  下述快照流程取证，避免同一趟产生重复订单。

“说得快”只是可能原因之一；必须先用状态、订单和语音日志区分“软件未就绪、没有最终识别
结果、订单未路由、状态机未接收”四类问题，不能仅凭车辆未动判断。

车已静止时，在任意 SSH 终端执行只读取证脚本：

```bash
cd ~/qhh
./scripts/mission/capture_factory_failure_snapshot.sh visual-rehearsal
```

它会输出：

```text
SNAPSHOT_READY=/home/ucar/qhh_failure_snapshots/<时间>_visual-rehearsal
```

把该目录、卡死时状态名和现场视频一起交给代码排查。快照包含 ROS 节点、关键话题、
控制发布者、系统资源和本轮日志，不会发布运动消息。

确认快照完成后停止本项目任务和语音栈：

```bash
cd ~/qhh
./scripts/mission/stop_factory_chain.sh visual-rehearsal
```

它不会停止底层导航、相机、ROS master 或独立巡线 launch。若已进入巡线：

1. 在终端 C 按 `Ctrl+C` 停止巡线；
2. 确认车轮静止；
3. 相机已因交接退出时，在终端 B 重新启动相机；
4. 重新启动终端 C 和 D，重新核对 AMCL，再开新趟次。

### 3.3 按状态定位问题

| 最后状态 | 先看什么 | 自动保障 | 本趟能否绕过 |
|---|---|---|---|
| `WAIT_ORDER` | `/question`、`/mission/order`、语音日志 | 可重复完整口令 | 不能手工造订单 |
| `NAVIGATING` | `/move_base/status`、AMCL、局部代价地图 | 失败后清代价地图并再试 1 次 | 不能遥控续跑 |
| `WAIT_QR` | `/vision/qr_items`、快照中的 QR 图 | 先走 8 个 `move_base` 45°静止视角；未集齐自动切换 `0.12 rad/s` 连续环视保底（最多 `390°/85 s`）；集齐立即停车 | 不能少于 3 码继续 |
| `WAIT_CLASSIFICATION` | `/mission/classification`、Spark 日志 | 8 秒超时，间隔 3 秒自动重试 | 不能伪造 Spark 结果 |
| `WAIT_SIM_CLASSIFICATION` | `/mission/sim_classification` | 同上 | 不能伪造结果 |
| `WAIT_WAREHOUSE` | `/vision/warehouse_marker`、相机曝光 | 每个厂牌最多 2 轮投票 | 不能猜厂牌 |
| `WAIT_SIMULATION_*` | link/ACK/progress/result 和请求 ID | 仿真桩或真实链按模式严格隔离 | 正式模式不能用桩 |
| `TASK4_READY` | traffic decision/gate/handoff 三个话题 | 停稳 3 秒、新鲜视觉、自动释放相机 | 不能手工发方向和 `88` |
| 巡线已开始 | 终端 C、`/visual_nav_end`、`/cmd_vel` 发布者 | 方向码重复 3 次，启动码 1 次 | 不能遥控 |
| `ANNOUNCING_TASK*` | announcement status、playback done | 任务 1–3 播放失败自动再试 1 次 | 未播完不能进下一阶段 |
| 终点不播报 | task5 completion status | 自动武装；真实结束+停稳后触发；失败再试 1 次 | 不能人工代播 |
| `ABORTED` | `reason` 和快照 `errors.txt` | 已取消目标并发零速 | 本趟不继续，修复后重开 |

## 4. 为什么不能“失败也强行跑到底”

本项目采用有界自动恢复，而不是伪造成功：

- QR 未读满三个，无法合法确定两件目标货品；
- 任务 1 播报未完成，规则不允许驶入生产区；
- 任务 2 播报未完成，规则不允许触发仿真；
- 任务 3 播报未完成，规则不允许进入停车区；
- 停止线条件未满足，不能进行交通决策；
- 播报期间必须锁止运动。

因此“完赛保障”是多视角 QR、导航重试、Spark 重试、OCR 投票、播报重试、仿真请求
校验和完整日志。所有合法重试仍失败后必须安全停车；跳过前置任务继续跑会同时失去
后续得分和合规证据。

## 5. 播报检查

固定顺序只有 `1 → 2 → 3 → 5`，任务 4 不播报：

| 任务 | 正式内容结构 |
|---|---|
| 1 | `取得X属于Y应放置在Z，仿真环境中取得A属于B应放置在C` |
| 2 | `已将X放入Z` |
| 3 | `仿真任务已完成，已将A放入C` |
| 5 | `任务完成` |

任务 1–3 使用本地 TTS，任务 5 使用已审核固定 WAV。状态机锁定本轮上下文、校验文本
哈希和任务顺序，只有播放成功回执后才允许继续。播放时车辆保持静止。

明天每趟重点核对：

```bash
rostopic echo /mission/announcement
rostopic echo /voice/playback_done
rostopic echo /mission/announcement_gate
```

若扬声器硬件无声但日志显示文本和软件播放成功，立即向裁判/工作人员报告并保留快照；
规则允许结合运行日志和视频核验，但不能赛后才补造日志。

## 6. 正式双机比赛流程（转发队友用）

两台电脑必须连接同一个局域网，但各自使用独立 ROS master。仿真端通过 TCP 主动连接
小车端 `<小车IP>:24580`。正式流程禁止使用 `visual-rehearsal` 仿真桩。

| 设备 | 负责人 | 主要工作 |
|---|---|---|
| 仿真端电脑 | Gazebo 队友 | 腾讯会议、Gazebo、RViz、导航、任务 Action、TCP bridge |
| 小车端电脑 | 实车队友 | SSH 小车、实体任务栈、TCP listener、状态检查、现场安全确认 |

### 6.1 两台电脑共同准备

1. 仿真端加入指定腾讯会议并共享完整桌面，Gazebo 与 RViz 必须全程同时可见。
2. 两端关闭休眠、锁屏和自动切网；禁止比赛中临时换热点。
3. 小车端执行 `hostname -I`，把局域网地址发给仿真端。当前现场示例为
   `192.168.31.112`，不要误用 `10.42.0.1`。
4. 仿真端验证：`ping -c 3 <小车IP>`。
5. 两端结束上一趟旧进程；不得同时启动两套 Gazebo、导航、bridge 或任务状态机。

### 6.2 初赛/省赛正式流程

**第一步，小车端先执行：**

```bash
ssh ucar@<小车IP>
cd ~/qhh
./scripts/mission/stop_preliminary_field_stack.sh competition
./scripts/mission/start_preliminary_field_stack.sh \
  --mode competition --arm-preliminary-field-stack
```

启动过程中车端会监听 `0.0.0.0:24580`。脚本若输出 `BLOCKED`，保留完整输出，不得说口令。

**第二步，仿真端执行：**

```bash
cd ~/gazebo_ws                    # 按实际工作空间路径修改
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch smart_factory_bringup full_competition.launch \
  start_navigation:=true start_gazebo:=true \
  start_navigation_server:=true start_perception:=true \
  start_bridge:=true vehicle_host:="192.168.31.112" vehicle_port:=24580
```

**第三步，双端检查：**

仿真端：

```bash
rostopic hz /clock
rostopic hz /scan
rostopic hz /amcl_pose
rostopic info /sim_task/execute/status
```

四项必须持续有数据或发布者。然后按官方原始流程重新运行一次未修改的
`spawn_cubes.py`，确认 Gazebo 底部 `real_time_factor <= 1.0`。

小车端：

```bash
cd ~/qhh
./scripts/mission/status_preliminary_field_stack.sh competition
```

必须同时看到 `PRELIMINARY_SOFTWARE_READY`、`PRELIMINARY_FIELD_READY`、`CHAIN_READY`，且
`/simulation/link_status` 顶层为 `connected=true, ready=true`。

### 6.3 全国总决赛正式流程

仿真端操作与 6.2 完全相同。小车端改用决赛脚本：

```bash
ssh ucar@<小车IP>
cd ~/qhh
./scripts/mission/stop_finals_field_stack.sh competition
./scripts/mission/start_finals_field_stack.sh \
  --mode competition --arm-finals-field-stack
```

启动后检查：

```bash
cd ~/qhh
./scripts/mission/status_finals_field_stack.sh competition
```

必须同时看到 `FINALS_SOFTWARE_READY`、`FINALS_FIELD_READY`、`CHAIN_READY`。除 6.2 的共同
条件外，决赛还必须确认：22° 上坡、平台和 25° 下坡区域清空；动态交通灯、前轮停止线
观察器和黄灯阻塞在线；四挡板、允许虚线出入区域和车体不压实线保护在线；急停人员就位。
任何决赛 blocker 未关闭都不得发车，也不得改用初赛脚本绕过。

### 6.4 全部 READY 后如何发车

裁判开始计时后，两台电脑都停止操作。只说一次第 1 节的完整订单。任务 2 播报并完成仿真
货品仓库停泊后，车端自动发送任务请求；仿真端自动 ACK、执行并回传阶段 20 结果；车端
自动播报任务 3 完成，再继续后续任务。中途不需要人工按键或发布 ROS 消息。

### 6.5 仿真端快速排障

| 现象 | 仿真端检查 | 处理 |
|---|---|---|
| `connect failed` | `ping <小车IP>`；确认 IP 和 24580 | 小车端先启动对应 `competition` 脚本；bridge 会自动重连 |
| `/clock` 无消息 | `rosnode list \| grep gazebo` | 确认 `start_gazebo:=true`，Gazebo 未退出且未使用另一个 ROS master |
| `/amcl_pose` 无消息 | 检查 `/amcl`、`/odom`、`/scan`、`odom→base_footprint` TF | 缺节点则重启完整 launch；输入齐全后在 RViz 按真实出生位设置 `2D Pose Estimate` |
| Action 不在线 | `rostopic info /sim_task/execute/status` | 确认 `start_navigation_server:=true` 且 mission launch 没有报错 |
| `ready=false` | 查看心跳五项 ready 和 `busy/fault_latched` | 缺哪项修哪项；禁止伪造 `/amcl_pose` 或完成回执 |
| RTF 超过 1 | 看 Gazebo 底部 `real_time_factor` | 降低非必要负载；不得以加速仿真换取成绩 |

若怀疑 bridge 与 Gazebo 不在同一个 ROS master，在**启动 bridge 的终端**执行：

```bash
echo "$ROS_MASTER_URI"
rosnode list | grep -E 'gazebo|amcl|move_base|sim_task|vehicle_bridge|rviz'
```

这些节点必须出现在同一 ROS master 中。

### 6.6 小车端快速排障与结束

| 现象 | 小车端处理 |
|---|---|
| 24580 未监听 | 重新执行对应阶段 `start_*_field_stack.sh --mode competition ...` |
| `connected=false` | 核对仿真端使用的小车 IP；确认两台电脑仍在同一局域网 |
| `connected=true, ready=false` | 网络已通，读取嵌套仿真心跳并让仿真端修复对应项 |
| 启动输出 `BLOCKED` | 保存完整输出；修复指定现场/配置/节点问题，禁止改标志绕过 |
| 任务中断 | 车辆静止后执行 `capture_factory_failure_snapshot.sh competition` |

结束本趟时，小车端按比赛阶段二选一：

```bash
./scripts/mission/stop_preliminary_field_stack.sh competition  # 初赛/省赛
./scripts/mission/stop_finals_field_stack.sh competition       # 全国总决赛
```

仿真端随后在唯一 launch 终端按 `Ctrl+C`。下一趟重新运行官方 `spawn_cubes.py`，不复用旧
订单、请求 ID、完成回执、AMCL 或物块状态。

## 7. 一趟结束后的最短重开流程

### 7.1 普通软件失败：复用底层，快速开新趟

```bash
cd ~/qhh
./scripts/mission/restart_factory_attempt.sh \
  --mode visual-rehearsal --arm-new-attempt
```

该命令仅接受 `ABORTED/COMPLETE`，或任务五已完成的 `TASK4_READY`；同时要求车辆停稳、
`move_base` 无活动目标。它并行保存故障快照，重建任务/语音层，复用健康的导航、AMCL
和巡线；若上一趟已释放 ROS 相机，会自动补回本脚本拥有的相机进程。

该快速重开只用于 `visual-rehearsal` 实验室测试。真实 Gazebo 的 `competition` 趟次必须
按 6.6 停止双端进程，再从 6.1 冷启动，避免复用旧 TCP 会话和仿真回执。

### 7.2 整车断电、ROS master 或底层导航退出

不能快速续跑。先机械固定电池仓与电源插头，把车放回发车位，再执行第 6 节的冷启动；
随后必须在 RViz 重设 AMCL，并重新运行 `status_finals_field_stack.sh`。断电前的
`run_id`、定位、订单、二维码、Spark 和仿真回执全部作废。

### 7.3 全部结束

```bash
cd ~/qhh
./scripts/mission/stop_finals_field_stack.sh visual-rehearsal
```

不要复用上一趟的 `ABORTED`、旧二维码、旧 Spark 结果或旧仿真回执。

### 7.4 赛前从手机热点切换到赛场路由器

只能在正式计时前切网；计时后不得触碰电脑、小车或路由器。当前车端
`ROS_MASTER_URI=http://localhost:11311`，且未设 `ROS_IP/ROS_HOSTNAME`，因此单纯 Wi-Fi 换网不要求重启 ROS 或整车冷启动。
切网会使旧 SSH 断开，车端 IP 也可能改变；仿真电脑到车端 `24580` 的 TCP 连接必须用新 IP 重建。

换网后不要立即说口令，按顺序复核：

1. 用赛场路由器分配的新 IP 重连 SSH；
2. 确认车和仿真电脑在同一赛场局域网，由仿真端改用车的新 IP；
3. 运行当前阶段的 `status_*_field_stack.sh`，确认底层节点和 `CHAIN_READY`未因断网退出；
4. 正式仿真模式还必须重新看到新鲜 `/simulation/link_status` ready；
5. 核对 AMCL、相机、雷达、里程计、Spark 域名解析与急停人员后，才进入发车准备。

只要小车没有断电、ROS master 和底层节点仍健康，上述检查全通过就不必重新冷启动。
若换网伴随整车重启、ROS master 退出、导航/相机节点丢失，或状态脚本不再输出就绪标记，则必须按 6.1/6.2 重新冷启动并重设 AMCL。

## 8. 交通灯左右识别与路径选择单独验收

本节只用于实验室模块联调，不是正式全链或决赛前轮越线验收。禁止使用
`traffic_rehearsal_decision_node` 的人工方向作为视觉通过证据。左、右必须分成两趟，每趟都从空栈、重新摆车和核对 AMCL 开始。

### 8.1 第一步：静态 A/B，严禁动车

车轮架空或底盘急停，交通灯放在正式位置，启动决赛排练栈但不说订单、不发
`/mission/stop_line_arrived` 或 `/mission/line_handoff_arm`：

```bash
cd ~/qhh
./scripts/mission/start_finals_field_stack.sh \
  --mode visual-rehearsal --arm-finals-field-stack
```

另一 SSH 窗口只读监视：

```bash
tail -F \
  ~/qhh_runtime/visual-rehearsal/traffic_vision.log \
  ~/qhh_runtime/visual-rehearsal/traffic_gate.log \
  ~/qhh_runtime/visual-rehearsal/line_handoff.log
```

按“红 2 s -> 左 3 s -> 红 2 s -> 右 3 s -> 红”显示。必须同时满足：

- 左转灯稳定输出 `green_left mode=1`；
- 右转灯稳定输出 `green_right mode=2`；
- 红灯输出 `red_light`，不得出现放行；
- `line_handoff` 始终 `armed=false/sent=false`，车轮始终不动。

若左右任一不符，到此停止，保存日志和图像，不得进入含底盘测试。

### 8.2 第二步：单方向含底盘模块联调

确认急停人员、站在停止线前、前方巡线/挡板/终点区完整清场，并且本趟只摆左或只摆右。
开始时交通灯保持红灯，先让安全门知道“已到停止线”：

```bash
source /opt/ros/noetic/setup.bash
rostopic pub -1 /mission/stop_line_arrived std_msgs/Bool 'data: true'
```

保持小车静止至少 3 s，再一次性武装交接：

```bash
rostopic pub -1 /mission/line_handoff_arm std_msgs/Bool 'data: true'
```

然后才让交通灯收到“左转”或“右转”口令。不得手工发 `/vision/traffic_decision`、`/visual_nav`、`88` 或 `/cmd_vel`。

日志监视：

```bash
tail -F \
  ~/qhh_runtime/visual-rehearsal/traffic_vision.log \
  ~/qhh_runtime/visual-rehearsal/traffic_gate.log \
  ~/qhh_runtime/visual-rehearsal/line_handoff.log \
  ~/qhh_runtime/finals-field/line.log
```

`traffic_gate` 与 handoff 的逐帧决策是 ROS 话题，不会每帧写入文本日志。需要精确核对时再开两个只读窗口：

```bash
rostopic echo /mission/traffic_gate
```

```bash
rostopic echo /mission/line_handoff_status
```

左转趟必须依次看到：

```text
green_left mode=1
/mission/traffic_gate: ok=true, command=green_left, line_mode=1
line handoff sent mode=1 then start=88
Mode switched to 1
Startup route started for mode 1
```

右转趟必须依次看到：

```text
green_right mode=2
/mission/traffic_gate: ok=true, command=green_right, line_mode=2
line handoff sent mode=2 then start=88
Mode switched to 2
Startup route started for mode 2
```

一趟结束后先取证，再停止全栈：

```bash
cd ~/qhh
./scripts/mission/capture_factory_failure_snapshot.sh visual-rehearsal
./scripts/mission/stop_finals_field_stack.sh visual-rehearsal
```

下一个方向必须重新摆车、重设/核对 AMCL、重新启动。若实际转向与上述任一层日志不一致，立即急停并保留现场，不得连续尝试另一方向。

## 9. 现场感知与 AI 模块快速验收（全程不动车）

用途：导航队友建图或调参期间，独立验证正式代码中的 AIUI 在线听写、Spark X2 指令拆解/货品分类、QR、OCR+ORB、交通灯和巡线模式选择。
本流程不启动任务状态机、巡线控制器或导航目标，只使用 `/field_test/*`、`/voice_test/*` 隔离话题；不能代替整车或规则验收。

### 9.1 安全前提与启动

1. 车辆必须停车；抱车前确认没有导航目标，队友不得同时发 move_base 目标；
2. 正式整链和正式语音栈必须停止；基础 ROS 与 `/camera/image` 保持在线；
3. 相机朝向待测目标，车轮不得因本流程转动。

一个终端启动隔离栈：

```bash
cd ~/qhh
./scripts/mission/start_field_module_acceptance.sh --arm-motion-isolated
```

必须看到 `FIELD_MODULE_ACCEPTANCE_READY`。第二个终端打开中文看板：

```bash
cd ~/qhh
python3 scripts/mission/watch_field_module_acceptance.py
```

### 9.2 五组模块测试

按顺序完成，避免同时把 QR、厂牌和灯具放进画面。

1. **AIUI + 订单拆解**：清晰、匀速说一遍完整订单。看板必须出现 `PASS AIUI_IAT` 和
   `PASS ORDER_PARSE`，且两个货品与口令一致。这只发布 `/voice_test/mission_order`，不会发车。
2. **Spark X2 分类**：示例命令：

   ```bash
   python3 scripts/mission/field_acceptance_spark_probe.py 食品 香蕉 手机 棉被
   ```

   必须出现 `PASS SPARK_CLASS`，示例结果应为香蕉。每次换三个互不相同的真实赛题货品再测一次。
3. **QR**：依次或同时让相机看到三张不同二维码；必须出现 `PASS QR`、`count=3`，货品名称和实体一致，无 identity conflict。
4. **OCR/ORB**：分别抱到 A/B/C 厂牌观察位置，每块牌等待至少 5 个处理周期；必须出现 `PASS OCR_ORB`、稳定票数至少 3，类别正确。
5. **交通灯与巡线选择**：按红、左、黄、红、右、黄、红、直行、黄、红逐态显示。必须看到：

   ```text
   red_light/yellow_light -> line_mode=None
   green_left             -> Trace_edge 1
   green_right            -> Trace_edge 2
   green_straight         -> Trace_edge 3
   ```

   看板的 `PASS LINE_SELECT` 只验证视觉结果到模式编号的选择，不发布 `/visual_nav`、`88` 或 `/cmd_vel`，因此车辆不会巡线。

任一模块失败时，保留看板文字并查看对应日志：

```bash
tail -F ~/qhh_runtime/field-module-acceptance/*.log
tail -F ~/qhh_runtime/field-module-acceptance/voice/*.log
```

### 9.3 结束与切回导航

```bash
cd ~/qhh
./scripts/mission/stop_field_module_acceptance.sh
```

必须看到 `FIELD_MODULE_ACCEPTANCE_STOPPED`。之后导航队友才可继续发目标；正式全链测试前仍须从空栈执行对应阶段的一键启动，不能复用
`/field_test/*` 或 `/voice_test/*` 的结果。
