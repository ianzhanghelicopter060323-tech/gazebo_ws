# 智慧工厂整链运行手册

更新：2026-08-20

本手册只保留现场会用到的启动、观察、止损、取证和重开命令。规则以
`第二十一届讯飞智慧工厂竞赛规则总手册.md` 为准；全国总决赛再叠加总决赛手册。

## 0. 当前比赛阶段

- 当前实体赛道是**初赛/省赛版本**，正式发车只使用第 1.4 节初赛命令。
- 决赛命令保留在第 1.5 节；只有决赛全部规则验收关闭后才能使用。
- 队友正在调整的导航、巡线和坡道参数以车端当前版本为准，本文档不修改这些参数。
- 正式计时后不操作电脑；唯一发车动作是说一次完整订单。

### 0.1 赛前 15 分钟现场调试唯一清单（初赛/省赛）

目标是验证本队负责的感知与任务链，不在 15 分钟内临时重调导航。按时间执行，前一项失败就
保存证据，不用后续时间盲目重复：

| 时间 | 操作 | 必须得到的结果 |
|---|---|---|
| 0–2 分钟 | `hostname -I`、电脑 `ping`、SSH；记录小车和仿真电脑 IP | 两机同网且 SSH 正常 |
| 2–4 分钟 | 启动基础 ROS/相机，再启动第 7 章隔离看板 | `FIELD_MODULE_ACCEPTANCE_READY` |
| 4–6 分钟 | 完整口令一次；Spark 分类探针一次 | 听到“唤醒单滴→成功单滴”；`PASS AIUI_IAT`、`PASS ORDER_PARSE`、`PASS VOICE_BEEP`、`PASS SPARK_CLASS` |
| 6–8 分钟 | 三张 QR；A/B/C 厂牌各观察一次 | QR `count=3`；三厂牌各稳定至少 3 票且类别正确 |
| 8–13 分钟 | 红、左、红、右、红、直行、黄、红；左右再随机复测一次 | 方向均稳定 5/5；左=1、右=2、直行=3；红/黄=None |
| 13–15 分钟 | 停止隔离栈，保存照片/帧/日志；启动正式栈只做 READY 检查 | `PRELIMINARY_SOFTWARE_READY`、`CHAIN_READY`；此时仍不发车 |

交通灯是最高优先级。若时间不足，宁可少做一次重复 QR/OCR，也必须完成左、右、直行、红、
黄及 1/2/3 映射检查。隔离看板不会发布 `/visual_nav`、`88` 或速度，车辆必须始终不动。

本次已把隔离方向 PASS 收紧为 5/5，并启用现场帧保存。取得 IP 后若车端还没有 2026-08-20
版本，先从 Mac 仓库根目录同步这四个隔离工具（不覆盖导航和正式巡线）：

```bash
CAR_IP=<小车IP>
scp scripts/mission/start_field_module_acceptance.sh \
    scripts/mission/watch_field_module_acceptance.py \
    scripts/mission/start_traffic_only_acceptance.sh \
    scripts/mission/watch_traffic_only_acceptance.py \
    "ucar@${CAR_IP}:/home/ucar/qhh/scripts/mission/"
ssh "ucar@${CAR_IP}" 'chmod +x /home/ucar/qhh/scripts/mission/start_*acceptance.sh /home/ucar/qhh/scripts/mission/watch_*acceptance.py'
```

### 0.2 封车前必须留下的现场资料

手机照片或视频至少包含：赛道全景、相机安装姿态、现场灯光/窗户反光、QR 实际距离、A/B/C
厂牌正视图，以及红、黄、左、右、直行五种灯态。文件名写清状态，例如：

```text
site_overview.jpg
camera_mount.jpg
qr_distance.jpg
warehouse_A.jpg  warehouse_B.jpg  warehouse_C.jpg
traffic_red.jpg  traffic_yellow.jpg
traffic_left.jpg traffic_right.jpg traffic_straight.jpg
traffic_random_sequence.mp4
```

相机原始视角可在每种目标保持静止时采 4 秒（只订阅图像，不控制车辆）：

```bash
FIELD_TAG=$(date +%Y%m%d_%H%M)
LABEL=traffic_right                     # 每次改成实际状态
mkdir -p ~/qhh_field_evidence/$FIELD_TAG/camera
cd ~/qhh_field_evidence/$FIELD_TAG/camera
timeout 4 rosrun image_view extract_images image:=/camera/image \
  _sec_per_frame:=0.5 _filename_format:="${LABEL}_%04i.jpg"
```

第 7 章隔离交通节点还会自动保留最近 240 张标注帧：
`~/qhh_runtime/field-module-acceptance/traffic_frames/`。现场结束前不得只留手机照片，必须同时
保留相机帧、看板输出和日志。

### 0.3 封车前把车端冻结到本地 Mac

在 **Mac 终端**执行。`CAR_IP` 改为现场地址；拉取时不要使用 `--delete`，并排除凭据文件：

```bash
CAR_IP=<小车IP>
FREEZE_TAG=$(date +%Y%m%d_%H%M)_field
FREEZE="/Users/grififth/Documents/Competitions/Smart car/iflytek-smart-car/ros/field_freeze/${FREEZE_TAG}"
mkdir -p "$FREEZE/qhh" "$FREEZE/UCAR_WS2026" "$FREEZE/evidence"

rsync -a --exclude '.env' --exclude '__pycache__' \
  "ucar@${CAR_IP}:/home/ucar/qhh/src/" "$FREEZE/qhh/src/"
rsync -a --exclude '__pycache__' \
  "ucar@${CAR_IP}:/home/ucar/qhh/scripts/" "$FREEZE/qhh/scripts/"
rsync -a "ucar@${CAR_IP}:/home/ucar/qhh/config/" "$FREEZE/qhh/config/"
rsync -a "ucar@${CAR_IP}:/home/ucar/qhh/data/vision_models/" "$FREEZE/qhh/vision_models/"
rsync -a --exclude '__pycache__' \
  "ucar@${CAR_IP}:/home/ucar/qhh/ros/UCAR_WS2026/src/" \
  "$FREEZE/qhh/ros/UCAR_WS2026/src/"
rsync -a --exclude '__pycache__' \
  "ucar@${CAR_IP}:/home/ucar/UCAR_WS2026/src/" "$FREEZE/UCAR_WS2026/src/"
rsync -a "ucar@${CAR_IP}:/home/ucar/qhh_runtime/field-module-acceptance/" \
  "$FREEZE/evidence/field-module-acceptance/"
rsync -a "ucar@${CAR_IP}:/home/ucar/qhh_field_evidence/" \
  "$FREEZE/evidence/qhh_field_evidence/"

# 若现场继续补充了原始交通灯数据集，再单独拉取；目录不存在时跳过。
ssh "ucar@${CAR_IP}" 'test -d /home/ucar/qhh/data/traffic_final_raw' && \
  rsync -a "ucar@${CAR_IP}:/home/ucar/qhh/data/traffic_final_raw/" \
  "$FREEZE/evidence/traffic_final_raw/"

find "$FREEZE" -type f ! -name SHA256SUMS.txt -exec shasum -a 256 {} \; \
  > "$FREEZE/SHA256SUMS.txt"
```

必须同时保存 `~/qhh` 的任务/视觉镜像和实际 catkin 工作空间 `/home/ucar/UCAR_WS2026/src`；
二者可能不同，不能只备份其中一个。确认 `SHA256SUMS.txt` 非空后再允许封车。不得复制或提交
`.env`、API 密钥、SSH 凭据。赛后离线修改应在本地副本/新分支完成，保留现场冻结目录只读；
比赛前再把明确文件复制回车端、编译并核对哈希，禁止整目录盲覆盖。

## 1. 正式比赛发车与恢复（现场唯一入口）

### 1.1 获取小车当前 IP

小车本机终端：

```bash
hostname -I
```

选择与仿真电脑同一局域网的地址。不要使用小车热点地址 `10.42.0.1`，除非两台电脑确实
都连接该热点。电脑验证：

```bash
ping -c 3 <小车IP>
ssh ucar@<小车IP>
```

### 1.2 仿真端启动（初赛、决赛相同）

仿真端先启动并保持终端运行；车端尚未监听时出现 `connection failed` 属正常重试，不要退出：

```bash
cd ~/gazebo_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch smart_factory_bringup full_competition.launch \
  start_navigation:=true start_gazebo:=true \
  start_navigation_server:=true start_perception:=true \
  start_bridge:=true vehicle_host:=<小车IP> vehicle_port:=24580
```

按官方流程运行未修改的 `spawn_cubes.py`，在 RViz 按真实出生位设置 AMCL。仿真端必须确认：

```bash
rostopic hz /clock
rostopic hz /scan
rostopic echo -n 1 /amcl_pose
rostopic info /sim_task/execute/status
```

合格输出：`/clock`、`/scan` 连续；AMCL 已初始化；Action status 有发布者；Gazebo
`real_time_factor <= 1.0`。

### 1.3 小车端登录

```bash
ssh ucar@<小车IP>
cd ~/qhh
```

一个 SSH 终端完成启动和检查，不需要分别启动导航、相机、语音和巡线。

### 1.4 初赛/省赛正式启动

```bash
./scripts/mission/stop_preliminary_field_stack.sh competition
./scripts/mission/start_preliminary_field_stack.sh \
  --mode competition --arm-preliminary-field-stack
./scripts/mission/status_preliminary_field_stack.sh competition
rostopic echo -n 1 /simulation/link_status
```

允许发车必须同时看到：

```text
PRELIMINARY_SOFTWARE_READY
PRELIMINARY_FIELD_READY
CHAIN_READY
connected=true
ready=true
busy=false
fault_latched=false
pending_request_id=""
```

### 1.5 全国总决赛正式启动

只有坡道、动态停止线、四挡板巡线避障及全部决赛预检已正式验收后执行：

```bash
./scripts/mission/stop_finals_field_stack.sh competition
./scripts/mission/start_finals_field_stack.sh \
  --mode competition --arm-finals-field-stack
./scripts/mission/status_finals_field_stack.sh competition
rostopic echo -n 1 /simulation/link_status
```

允许发车必须同时看到：

```text
FINALS_SOFTWARE_READY
FINALS_FIELD_READY
CHAIN_READY
connected=true
ready=true
busy=false
fault_latched=false
pending_request_id=""
```

任何决赛 `BLOCKED` 都必须按提示修复，禁止改用初赛脚本绕过。

### 1.6 人工检查与唯一发车口令

READY 后还要人工确认：车辆在发车格且对正、AMCL 与实车一致、电池仓固定、电量充足、
赛道清空、急停人员就位。裁判开始计时后只说一次：

```text
小飞小飞，前往物品领取区，取得<实体货品>，放置在对应仓库，并领取仿真环境中需要的<仿真货品>放置在对应仓库
```

状态机仍为 `READY` 且 `/mission/order` 尚未锁存时，才允许清晰重说一次。

现场声学反馈固定为：唤醒成功一声短滴；完整订单通过 AIUI、Spark X2 和本地严格
校验后一声短滴；识别或拆解失败两声快速短滴。听到第一声后立即说订单正文；听到
第二声单滴表示订单已锁存，程序在约 0.09 秒提示音结束后才发布 `/mission/start`，
随后紧接运动。两声快滴表示本次未发车且允许重新唤醒，不能把它误判为成功。

### 1.7 SSH 断联

SSH 断开不会停止节点。重连后不要再次 start，只检查当前阶段：

```bash
cd ~/qhh
./scripts/mission/status_preliminary_field_stack.sh competition   # 初赛
# ./scripts/mission/status_finals_field_stack.sh competition      # 决赛
rostopic echo -n 1 /simulation/link_status
```

READY 与链路均正常即可继续；否则执行第 1.9 节完整重开。

### 1.8 更换网络环境

只允许正式计时前换网。重新执行第 1.1 节取得新 IP；仿真端 `Ctrl+C` 停止旧 launch，使用
新 IP 重新执行第 1.2 节。车端重连后先检查状态和链路。若 READY 全在且链路恢复，不必重启
小车；否则执行第 1.9 节。

### 1.9 中途失败、主动结束或一趟结束后重开

先确保车辆静止。失败时先取证，正常完成可跳过快照：

```bash
cd ~/qhh
./scripts/mission/capture_factory_failure_snapshot.sh competition
./scripts/mission/stop_preliminary_field_stack.sh competition      # 初赛
# ./scripts/mission/stop_finals_field_stack.sh competition         # 决赛
```

仿真端 `Ctrl+C`，重新运行未修改的 `spawn_cubes.py`、重设 AMCL，并重新执行第 1.2 节。
车端按比赛阶段重新执行第 1.4 或 1.5 节。不得复用旧订单、二维码、TCP request、pending 或
仿真完成回执。

若整车断电、ROS master、导航或相机退出，固定电池仓并把车放回发车位后，也必须执行本节
完整重开，不能只重启任务节点。

### 1.10 运行中整车断电后的固定恢复流程

正式计时中断电：不要碰车或电脑，先报告裁判并按裁判口径结束本趟；获准后再处理。实验室
调试则先确认底盘完全失电且不会突然恢复。断电会同时丢失 ROS master、AMCL、订单锁存、
节点内存和 TCP 会话，因此**禁止从中断任务续跑，禁止只重启任务节点，禁止直接重说口令**。

1. 断开电池，检查并固定电池仓、插头、保险和供电线；更换电量充足电池。检查无短路、发热、
   松脱或机械损伤后，把车放回合法发车位并对正，再重新上电。
2. 等小车系统完整启动，在小车本机执行 `hostname -I`，按第 1.1 节确认 IP 后重新 SSH。若 IP
   改变，仿真端必须同步使用新 IP。
3. 仿真端按 `Ctrl+C` 停止旧 launch；重新启动 Gazebo/RViz/bridge，重新运行官方未修改的
   `spawn_cubes.py`，重新设置并核对仿真 AMCL。不得复用断电前 pending、request 或完成回执。
4. 车端执行对应阶段的完整停止和冷启动：

```bash
cd ~/qhh

# 初赛
./scripts/mission/stop_preliminary_field_stack.sh competition
./scripts/mission/start_preliminary_field_stack.sh \
  --mode competition --arm-preliminary-field-stack
./scripts/mission/status_preliminary_field_stack.sh competition

# 决赛时改用以下三条，不与初赛混用：
# ./scripts/mission/stop_finals_field_stack.sh competition
# ./scripts/mission/start_finals_field_stack.sh --mode competition --arm-finals-field-stack
# ./scripts/mission/status_finals_field_stack.sh competition

rostopic echo -n 1 /simulation/link_status
```

5. 重新核对实体车 AMCL、`/scan`、`/odom`、相机、电池固定及第 1.4/1.5 节全部 READY 条件。
   只有 `pending_request_id=""`、`busy=false`、`fault_latched=false` 后，才可开始**全新趟次**并
   重新说完整口令。任一项不满足不得发车。

## 2. 不含真实仿真的测试流程（与正式流程隔离）

本节只用于测试实体任务链和任务 3 接口时序，使用 `visual-rehearsal` 的 5 秒仿真桩，
不连接 Gazebo，也不建立真实 TCP 仿真链路。它不能作为任务 3 正式验收证据，且不能与
第 1 章的 `competition` 正式流程同时运行。

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

## 6. 交通灯含底盘专项验收（15 分钟标准流程默认不做）
本节只在场地允许车辆运动、急停和巡线区域完整清场时使用，不是第 0.1 节的标准静态流程，
也不是正式全链或决赛前轮越线验收。禁止使用
`traffic_rehearsal_decision_node` 的人工方向作为视觉通过证据。左、右必须分成两趟，每趟都从空栈、重新摆车和核对 AMCL 开始。

### 6.1 第一步：静态 A/B，严禁动车

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

### 6.2 第二步：单方向含底盘模块联调

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

## 7. 现场感知与 AI 模块快速验收（全程不动车）

用途：导航队友建图或调参期间，独立验证正式代码中的 AIUI 在线听写、Spark X2 指令拆解/货品分类、QR、OCR+ORB、交通灯和巡线模式选择。
本流程不启动任务状态机、巡线控制器或导航目标，只使用 `/field_test/*`、`/voice_test/*` 隔离话题；不能代替整车或规则验收。

### 7.1 安全前提与启动

1. 车辆必须停车；抱车前确认没有导航目标，队友不得同时发 move_base 目标；
2. 正式整链和正式语音栈必须停止；基础 ROS 与 `/camera/image` 保持在线；
3. 相机朝向待测目标，车轮不得因本流程转动。

若车端尚未部署声学反馈版本，先在 **Mac 仓库根目录**执行以下定向部署；它只
备份并更新语音反馈、语音启动脚本和本节看板，不覆盖导航、视觉模型或凭据：

```bash
./scripts/deploy_voice_feedback_to_car.sh \
  --car-ip <小车IP> \
  --arm-deploy
```

看到 `VOICE_FEEDBACK_DEPLOY_OK` 后再登录小车执行本节命令。部署本身不会启动节点。

若导航队友已经启动基础 ROS 和相机，不要重复执行。否则先在两个终端分别启动，期间不发送
导航目标：

```bash
# 终端 A：基础 ROS、雷达、定位、move_base 待机
source /opt/ros/noetic/setup.bash
source /home/ucar/UCAR_WS2026/devel/setup.bash
roslaunch ucar_nav2026 ucar_navigation2026.launch
```

```bash
# 终端 B：ROS 相机
source /opt/ros/noetic/setup.bash
source /home/ucar/UCAR_WS2026/devel/setup.bash
rosrun ucar_camera ucar_camera.py
```

看到 `/scan`、`/odom`、`/camera/image` 有连续数据后再启动隔离栈。若端口或节点冲突，说明已有
基础组件，停止新开的重复进程，不要叠加启动。

一个终端启动隔离栈：

```bash
cd ~/qhh
./scripts/mission/start_field_module_acceptance.sh --arm-motion-isolated
```

必须看到 `FIELD_MODULE_ACCEPTANCE_READY`。第二个终端打开中文看板：

```bash
cd ~/qhh
FIELD_TAG=$(date +%Y%m%d_%H%M)
mkdir -p ~/qhh_field_evidence/$FIELD_TAG
python3 scripts/mission/watch_field_module_acceptance.py | \
  tee ~/qhh_field_evidence/$FIELD_TAG/module_watch.txt
```

### 7.2 五组模块测试

按顺序完成，避免同时把 QR、厂牌和灯具放进画面。

1. **AIUI + 订单拆解及声学反馈**：先说“小飞小飞”。麦克风阵列确认唤醒后，
   扬声器必须立即发出**一声短滴**；听到短滴后马上清晰、匀速说完整订单正文。
   订单通过 AIUI、Spark X2 和本地严格校验后必须再发出**一声短滴**。若识别、
   句式校验或 Spark 拆解失败，必须发出**两声快速短滴**，此时订单不得锁存，允许
   重新说“小飞小飞”开始下一次尝试。看板必须依次出现：

   ```text
   INFO VOICE_BEEP   唤醒单滴 ok=True
   PASS AIUI_IAT     <完整听写文本>
   PASS ORDER_PARSE  real=<实体货品> sim=<仿真货品>
   PASS VOICE_BEEP   成功单滴 ok=True
   ```

   单滴音频约 0.09 秒，失败双滴音频约 0.165 秒。唤醒和失败提示异步播放；成功
   提示位于“订单已发布、启动信号尚未发布”的安全间隙，提示结束后立即发布启动
   信号，固定增加约 0.09 秒而不是整句语音。两声快速短滴只表示本次失败可重试，
   绝不能被当作发车成功。第 7 章只发布 `/voice_test/mission_order`，不会发车。
2. **Spark X2 分类**：示例命令：

   ```bash
   python3 scripts/mission/field_acceptance_spark_probe.py 食品 香蕉 手机 棉被
   ```

   必须出现 `PASS SPARK_CLASS`，示例结果应为香蕉。每次换三个互不相同的真实赛题货品再测一次。
3. **QR**：依次或同时让相机看到三张不同二维码；必须出现 `PASS QR`、`count=3`，货品名称和实体一致，无 identity conflict。
4. **OCR/ORB**：分别抱到 A/B/C 厂牌观察位置，每块牌等待至少 5 个处理周期；必须出现 `PASS OCR_ORB`、稳定票数至少 3，类别正确。
5. **交通灯与巡线选择（最高优先级）**：按红、左、红、右、红、直行、黄、红逐态显示，
   每个方向保持到看板显示 `votes=5`；然后随机交换左/右顺序再复测一次。必须看到：

   ```text
   red_light/yellow_light -> line_mode=None
   green_left             -> Trace_edge 1
   green_right            -> Trace_edge 2
   green_straight         -> Trace_edge 3
   ```

   方向只有稳定 5/5 才显示 `PASS LINE_SELECT`。红、黄必须始终 `line_mode=None`；若左右任一
   次弄反、在静止灯态间跳变，或直行达不到 5/5，立即保存帧和日志，本次交通灯验收失败。
   看板只验证视觉结果到模式编号的选择，不发布 `/visual_nav`、`88` 或 `/cmd_vel`，因此车辆不会巡线。

任一模块失败时，保留看板文字并查看对应日志：

```bash
tail -F ~/qhh_runtime/field-module-acceptance/*.log
tail -F ~/qhh_runtime/field-module-acceptance/voice/*.log
```

### 7.3 结束与切回导航

```bash
cd ~/qhh
./scripts/mission/stop_field_module_acceptance.sh
```

必须看到 `FIELD_MODULE_ACCEPTANCE_STOPPED`。之后导航队友才可继续发目标；正式全链测试前仍须从空栈执行对应阶段的一键启动，不能复用
`/field_test/*` 或 `/voice_test/*` 的结果。

若第 7.1 节由你临时手工启动了终端 A/B，先分别 `Ctrl+C` 停止导航和相机，并确认相关节点
退出；否则正式一键脚本会因检测到非本脚本的 `/move_base` 或 `/ucar_camera` 而正确拒绝叠加。

结束后立即执行第 0.2、0.3 节保存现场帧、日志和两套车端源码。正式比赛软件只在第 1.4 节
完整预检通过后才算待发；隔离看板全部 PASS 不能替代 `PRELIMINARY_SOFTWARE_READY`。
