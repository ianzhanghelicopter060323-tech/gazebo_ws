# 正式比赛 Full Chain：预赛与全国总决赛分阶段发车手册

更新：2026-08-23
适用范围：基础/预赛与全国总决赛。两个阶段的发车命令分别冻结在第 5、6 章，禁止交叉使用。

现场快速入口：

- 预赛/省赛：只看第 5 章；
- 全国总决赛：只看第 6 章；
- 两阶段共用的订单、双机仿真和日志说明：看第 1、4、7 章。

## 1. 冻结合同

正式口令只有这一种格式：

```text
小飞小飞，前往物品领取区，取得<实体目标大类>，放置在对应仓库，并领取仿真环境中需要的<仿真目标大类>，放置在对应仓库
```

两个槽位只能是 `食品 / 日用品 / 电子产品`；第二个大类后必须继续说最后的
`放置在对应仓库`。示例：

```text
小飞小飞，前往物品领取区，取得食品，放置在对应仓库，并领取仿真环境中需要的电子产品，放置在对应仓库
```

任务一数据合同：

- QR 网页只接受 JSON object：`{"code":200,"result":"具体货品名"}`；
- 必须锁存本轮三个不同 QR 具体货品；
- 实体与仿真各调用一路 Spark X2，从这三个候选中选择属于口令大类的具体货品；
- Spark 结果必须仍在本轮三码候选内，类别必须等于口令大类；
- 任务一播报格式固定为：
  `取得[实体货品]属于[实体大类]应放置在[实体车间]，仿真环境中取得[仿真货品]属于[仿真大类]应放置在[仿真车间]`。

不得用默认货品、旧订单、猜测 QR、固定随机厂区坐标或人工巡线方向补失败。

二维码的“随机”由组委会网页/API 返回，不由车端伪造。三个网页分别对应食品、日用品、
电子产品大类，网页在对应大类内返回随机具体子类；二维码在领取区的摆放位置也随机。
这不表示车端应对同一二维码反复请求来重新抽取。正式链每次新进入 `WAIT_QR` 都清空上一趟
HTTP 成功缓存；同一趟内每个二维码只采用首次 `code=200` 的 `result`，重复视频帧不会
重新抽货品。三个二维码稳定集齐后立即锁存并进入两路 Spark 分类，不等待固定扫描时长。
三个码不必同时出现在一帧，节点按二维码身份跨视角累计、记忆 45 秒；本地生成的样例二维码
指向固定 JSON，所以重复测试得到固定货品是正常现象。

2026-08-22 对现场远景照片做了不联网离线复核：照片中只出现一个可见二维码；全图快速
OpenCV 未命中，现役五块重叠裁剪的中央裁块和深度兜底均解出同一 `http` URL。说明当前
裁剪兜底覆盖该视角，但手机远景照片不等于车载相机动态验收。比赛前仍须用实际
`/camera/image` 依次验证三块二维码、网页 JSON 和三码锁存，不能用这一张照片宣称 3/3 通过。

同日新增三张录像抽帧 `IMG_8615/8616/8617`，仍只做离线解码、不访问网页：`8616` 可由正式
重叠裁剪路径解出一个 HTTP URL，`8617` 可由全图和裁剪路径解出另一个 HTTP URL；`8615`
因二维码像素过小并带运动压缩，OpenCV、临时 zxing-cpp 和深度路径均未解出。三张都是场地
总览而不是车载近景，所以该结果证明“二维码内容格式和两个现场样本可读”，不证明远景
`3/3`。正式策略不因一张不可恢复的远景帧启用逐帧深度扫描，以免拖慢环视；必须靠车辆的
八个观察角、跨视角 45 秒记忆和近距离裁剪取得清晰帧。赛前若 `/camera/image` 的近景仍像
`8615` 一样模糊，应先处理曝光、对焦、距离或停车抖动，不能靠放宽 JSON/Spark 合同猜结果。

## 2. 当前两种策略

### 第一轮：score-first

- 正常完成任务一；
- 明确不搜索、不识别、不播任务二；
- 不使用旧 `factory_a/b/c` 固定坐标；最多用 45 秒搜索“仿真目标大类”对应的随机车间；
- 搜索成功则在该车间停稳后请求真实仿真；
- 若只以 `search_timeout/search_exhausted/target_not_found` 失败，则在当前新鲜 AMCL 安全停稳位
  继续请求仿真，优先尝试任务 3.2 和后续任务；
- 仿真请求中的货品、类别和目标仓库始终来自本轮 QR＋Spark；
- 完成真实仿真后继续任务四、任务五。

若 45 秒内找到正确车间，仍可争取任务 3.1 和 3.3；若搜索失败，仿真协议本身仍可运行，
但任务 3.1 位置分和任务 3.3“指定仓库原地播报”不能宣称合格。固定去旧 A 点看似只放弃
5 分，实际上随机摆放后该点可能不是任何仓库，并且会引入固定坐标运动的规则与安全风险，
因此正式代码不提供该入口。

### 第二轮：dynamic-task2

- 运动期间 OCR 持续处理最新帧；
- 高置信单帧只取消当前 move_base 目标并停车，不直接认定车间；
- 停稳后用新帧做 2/5 初筛；
- 按文字框中心修正朝向；
- 仅对雷达拟合出的长直线墙面做低速闭环靠近；
- 重新开启 OCR 会话，以 3/7 新帧终审；
- 实体与仿真目标共享 120 秒，不会各获得 120 秒。

`random_factory_search.yaml` 中
`request.parking_geometry_verified` 默认是 `false`。OCR、文字对中和雷达靠墙成功
只能证明“找到了车间并靠近”，不能自动证明三轮已完整进入停车区。只有同版本实车
完成停车区验收后才能改成 `true`；否则状态机会跳过任务二播报，但仍继续搜索仿真
目标并尝试任务三至五。

允许降级继续的只有：

```text
search_timeout
search_exhausted
target_not_found
parking_geometry_unverified
```

ROS、AMCL、视觉节点、控制器或协议身份不一致仍安全中止。

### 第二轮 OCR 当前版本

保留本地现版，不回退到远程分支的宽松参数：

- 只输出 `FOOD / DAILY / ELECTRONIC / unknown`；“科大讯飞、产教融合、iflytek”等不映射；
- 完整仓名、跨行部分仓名，以及单独出现的“食品/日用品/电子产品”均可作为候选；
- 短类别词仍须通过 OCR 置信度、类别分差和外层多帧投票，多个类别同帧时拒识；
- 模糊匹配只容忍约一个 OCR 错字，相似度下限 `0.80`；
- 完整仓名且置信度 `>=0.92` 时走 OCR 快速路径，否则保留 ORB 降级与冲突拒识。

现有离线证据：仓库数据集 ORB 正样本 `168/168`、负样本误报 `0/124`；三张正式模板的
合成遮挡在 10%～30% 时 `15/15`，40% 时 `12/15` 且其余拒识、没有错分。大字体通常会提高
OCR 可读性，但不能消除运动模糊、反光、斜视和真实锥桶遮挡；这些数字不是正式场地动态搜索
成功率。两轮若搜索随机车间都会使用本版本；第一轮不执行也不播报任务二，但仍可能用 OCR
限时寻找仿真目标车间。

### 任务一至任务三接口核对

正式数据链保持单向、按本趟身份锁存：

```text
/question 原始 ASR
-> voice_start_bridge 严格校验完整句式和两个大类
-> /mission/order（先发布）
-> 成功短滴完成
-> /mission/start（后发布）
-> QR 三个身份的首次 code=200 结果
-> /mission/qr_candidates
-> 两路真实 Spark X2 分类
-> announcement context / 固定格式任务一播报
-> score-first 仿真目标搜索，或 dynamic-task2 实体+仿真搜索
-> /simulation/mission_request
-> ACK / progress / stage=20 result
-> 固定格式任务三播报
```

不完整口令、非三大类、旧订单、二维码 HTTP/JSON 失败、Spark 返回不在三码候选内、请求身份
不一致或仿真未到 stage 20 均不会被伪装成成功。OCR 的两种比赛策略不是两套不兼容接口：
`score-first` 跳过任务二得分，但仍通过同一动态 OCR/雷达请求寻找仿真目标；
`dynamic-task2` 在同一接口上先找实体目标、再找仿真目标，共享 120 秒。两者都用
`session_id + request_id + order_id + target_label` 绑定结果，旧会话结果不能污染新趟次。

## 3. Mac 本地同步到小车

先获得小车当前 IP，并确认连通：

```bash
ping -c 2 <小车IP>
ssh ucar@<小车IP>
```

在 Mac 项目根目录执行唯一部署命令：

```bash
cd "/Users/grififth/Documents/Competitions/Smart car/iflytek-smart-car"
./scripts/deploy_preliminary_competition_chain_to_car.sh \
  --car-ip <小车IP> \
  --arm-deploy
```

成功必须看到：

```text
PRELIMINARY_CHAIN_STAGING_CHECK_OK
PRELIMINARY_CHAIN_FILE_COMPARE_OK
PRELIMINARY_CHAIN_OFFLINE_CHECK_OK
PRELIMINARY_CHAIN_DEPLOY_OK
未同步任何 .env/凭据；未启动 ROS、语音、导航或车辆
```

脚本会建立一次复用 SSH 会话，先在 staging 中校验，再备份、安装并逐文件字节比较：

- `/home/ucar/qhh`：完整初赛运行闭包（任务 1～5、Spark、QR、OCR、交通灯、播报、
  双发车、监控、模型和模板）；
- `/home/ucar/voice_ws_2026`：完整正式语音 Python 模块和语音启停脚本；
- `/home/ucar/UCAR_WS2026/src/ucar_nav2026`：随机厂区控制器、launch、YAML 和安全导航入口。

它不会同步 `.env` 或任何凭据；不会覆盖队友的坡道、决赛巡线或其他导航参数。部署前若
检测到任务/随机厂区进程、空间少于 150 MiB、缺目录或 staging 校验失败，会直接拒绝。
安装阶段异常会尝试恢复备份。回滚包在：

```text
/home/ucar/deploy_backups/<时间>_preliminary_chain/
```

“SSH 已连上”是必要条件但不是全部条件：Mac 与车须网络互通，三个车端工作区和既有 ROS/
Python 依赖须存在，部署时必须空栈。上述脚本会逐项检查；只有四个成功标志全部出现才算
同步完成。本轮仅 Python、shell、配置、模型和 launch 资源，不需要 `catkin_make`。

## 4. 发车前一次性检查

先取得小车本轮局域网 IP。小车端清理旧趟次：

```bash
ssh ucar@<小车IP>
cd /home/ucar/qhh
source scripts/mission/refresh_ros_network_env.sh
./scripts/mission/stop_preliminary_field_stack.sh competition
```

若刚换网络，先确认：

```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
hostname -I
```

不要手工沿用旧 IP。正式一键脚本会再次执行 `refresh_ros_network_env.sh`。

仿真电脑与小车接入同一局域网。在仿真电脑启动唯一完整栈；工作空间若不是
`~/gazebo_ws`，把下列两处 `~/gazebo_ws` 都替换成实际路径：

先确认网络和仿真 bridge 版本。`<小车IP>` 必须是当前同一局域网内的车端地址：

```bash
ping -c 2 <小车IP>
source /opt/ros/noetic/setup.bash
source ~/gazebo_ws/devel/setup.bash
bridge_pkg="$(rospack find smart_factory_bridge)"
if grep -n 'is_server_connected' "${bridge_pkg}/scripts/vehicle_bridge_node.py"; then
  echo 'BLOCKED: 仿真 bridge 还是旧版 actionlib API'
else
  grep -n 'wait_for_server' "${bridge_pkg}/scripts/vehicle_bridge_node.py"
fi
```

必须没有 `is_server_connected`，并能看到 `wait_for_server`。否则仿真电脑运行的不是本仓库
已经修正的 bridge，虽然 TCP 可能连上，真正收到任务时仍会失败。

```bash
cd ~/gazebo_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch smart_factory_bringup full_competition.launch \
  start_navigation:=true start_gazebo:=true \
  start_navigation_server:=true start_perception:=true \
  start_bridge:=true vehicle_host:=<小车IP> vehicle_port:=24580
```

仿真 bridge 是主动重连客户端。此时小车正式 listener 尚未启动，短暂出现
`connection failed` 正常；保持该 launch 运行，不启动第二套 bridge。仿真端另开只读终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/gazebo_ws/devel/setup.bash
rostopic hz /clock
rostopic hz /scan
timeout 5 rostopic echo -n 1 /amcl_pose
rostopic info /sim_task/execute/status
rosnode list | grep -E 'rviz|smart_factory_vehicle_bridge|mission'
```

应有持续 `/clock`、`/scan`，定位至少真实初始化一次，Action status 有发布者，且 RViz、
bridge、mission 节点存在。真正的 TCP `connected` 要等第 5/6 节小车 listener 上线后判断。

这套结构不共享 ROS master：小车是 `0.0.0.0:24580` 的 TCP 服务端，仿真 bridge 是自动重连
客户端，两端各自保留 ROS master。因此仿真可先启动，看到短暂 `connection failed` 后等待
小车；也可小车先启动再开仿真。不存在“仿真必须等小车软件全部启动后才允许 launch”的
架构依赖，但只有两边都完成后才会 ready，且只能运行一套 bridge。

小车 listener 上线后，用以下机器可判定的检查代替肉眼搜字符串：

```bash
cd /home/ucar/qhh
if timeout 5 rostopic echo -n 1 /simulation/link_status | \
  /usr/bin/python3 -m src.mission.simulation_protocol --check-link-status-echo; then
  echo SIMULATION_LINK_READY
else
  echo SIMULATION_LINK_NOT_READY
fi
```

`SIMULATION_LINK_READY` 证明同网段 TCP、双向心跳、Gazebo、RViz、Action server、AMCL 和
雷达就绪；它不等于机械臂任务已经执行过。要验证“指令能读到并完成”，必须在正式计时前
做一次真实仿真请求并看到 ACK、progress、最终 `success=true/completed_stage=20`，随后重启
仿真并按规则重新运行原版 `spawn_cubes.py`，再开始正式轮次。Mac 只负责 SSH/看日志即可，
不需要加入两套 ROS master；Mac 不能替代 Ubuntu 上真实 Gazebo+RViz 的规则验收。

## 5. 预赛/省赛 Full Chain 发车章

本章只用于基础/预赛赛道。全国总决赛不得运行本章命令，必须跳到第 6 章。

### 5.1 第一轮：保任务 1/3/4/5

小车终端只运行：

```bash
cd /home/ucar/qhh
./scripts/mission/start_monitored_competition_run.sh \
  score-first --arm-monitored-competition-run
```

成功时必须看到以下三项；由于内层栈先完成就绪，终端通常先打印
`PRELIMINARY_SOFTWARE_READY`，再打印后两项：

```text
PRELIMINARY_SOFTWARE_READY
MONITORED_RUN_READY: score-first
MONITORED_STARTUP_SECONDS: <本机实际秒数>
```

这三项只表示小车软件栈已待机。此时再在小车端检查真实双机链路：

```bash
rostopic echo -n 1 /simulation/link_status
```

必须看到顶层 `connected=true`、`ready=true`、`degraded=false`、空
`pending_request_id`；嵌套仿真状态必须有 `gazebo_ready=true`、`rviz_ready=true`、
`action_server_ready=true`、`localization_ready=true`、`laser_ready=true`、`busy=false`。
若实际版本还有 `fault_latched`，必须为 `false`。否则不要说口令。

车端尚无同版本冷启动计时证据。按脚本等待结构，健康冷启动的现场计划值是约 `20～45 秒`；
最终只认 `MONITORED_STARTUP_SECONDS` 实测值。超过 45 秒先看终端正在等待哪个节点，不要说
口令；各项硬超时累计可接近 3 分钟，触发后会明确 `BLOCKED` 并回收本轮组件，不能把超时
当作 READY。

本地离线预检当前为 `PASS`、`blockers=[]`；动态任务二未实车验收只产生 warning，第一轮
不会因此被挡住。仍保留必要安全门：已有整链进程、导航/AMCL/相机/雷达缺消息、随机搜索
控制器未订阅、Spark 配置不完整、AIUI/麦克风未就绪、组件退出或状态机未到 READY，任一项
都必须 `BLOCKED`。因此不能承诺任何硬件/网络状态下永不 BLOCKED；按第 4 节空栈启动且实际
依赖正常时，不再有旧版 `config_not_verified` 一类静态占位门阻止第一轮。

再人工确认车辆在 P 点、AMCL 正确、电池和急停正常、赛道无人、仿真 ready，然后只说
一次第 1 节完整口令。

预期主线：

```text
任务一 -> TASK2_SCORE_FIRST_BYPASS
-> 最多 45 秒搜索仿真目标车间
-> 找到目标车间，或搜索类失败后在当前位停稳降级
-> 真实仿真成功与任务三播报
-> 黄线前停止 -> 交通灯视觉 -> 巡线 -> 任务五播报
```

### 5.2 第二轮：尝试动态任务二

上一轮必须先完整停止，并把车放回 P 点、重新核对 AMCL：

```bash
cd /home/ucar/qhh
./scripts/mission/stop_preliminary_field_stack.sh competition
```

第二轮启动：

```bash
cd /home/ucar/qhh
./scripts/mission/start_monitored_competition_run.sh \
  dynamic-task2 --arm-monitored-competition-run
```

必须看到：

```text
PRELIMINARY_SOFTWARE_READY
MONITORED_RUN_READY: dynamic-task2
MONITORED_STARTUP_SECONDS: <本机实际秒数>
```

随后仍须按本章 5.1 的方式执行 `/simulation/link_status` 检查，再说完整正式口令。若停车几何验收
标志仍为 `false`，任务二不会播“已完成”，这是
规则安全门，不是任务一或 OCR 故障。

### 5.3 预赛停止与日志

预赛日志监视：

```bash
cd /home/ucar/qhh
./scripts/mission/status_full_chain_monitor.sh --follow
```

预赛正常停止：

```bash
cd /home/ucar/qhh
./scripts/mission/stop_preliminary_field_stack.sh competition
```

日志目录后缀分别为 `_score-first`、`_dynamic-task2`。更完整的事件解释见第 7 章。

## 6. 全国总决赛 Full Chain 发车章

本章只用于全国总决赛，不使用第 5 章的 `score-first` 或 `dynamic-task2` 发车命令。

国赛路线已经冻结为：

```text
正式完整订单
→ 发车点 → 坡道入口 → 22°上坡 → 平台 → 25°下坡 → 物品领取区
→ QR / 双 Spark / 任务一播报
→ 任务二 → 真实仿真 → 动态交通灯 → 巡线挡板避障 → 终点播报
```

现有 `deploy_preliminary_competition_chain_to_car.sh` 只部署预赛闭包，不同步
`ucar_nav_finals` 和 `visual_navigation_finals2`。国赛执行本章前，必须先确认本地修改已经同步到
`/home/ucar/qhh` 与 `/home/ucar/UCAR_WS2026` 对应决赛目录，并完成需要的 catkin 构建；不能仅运行
预赛部署脚本后就认为国赛代码已更新。

### 6.1 国赛开局前清理

```bash
ssh ucar@<小车IP>
cd /home/ucar/qhh
source scripts/mission/refresh_ros_network_env.sh
./scripts/mission/stop_monitored_finals_run.sh
```

把车辆放回国赛发车点，核对 AMCL、坡道入口方向、电池固定、急停、场地和仿真电脑
`SIMULATION_LINK_READY`。停止命令会中止坡道、取消 move_base、停止 finals2 并归档上一趟日志。

### 6.2 国赛唯一一键待机命令

小车终端只运行：

```bash
cd /home/ucar/qhh
./scripts/mission/start_monitored_finals_run.sh --arm-monitored-finals-run
```

必须依次看到：

```text
FINALS_SOFTWARE_READY
MONITORED_FINALS_READY
MONITORED_STARTUP_SECONDS: <本机实际秒数>
```

启动过程中还会输出：

```text
FINALS_STARTUP_STAGE  <阶段>  elapsed_s=<秒>
CHAIN_STARTUP_STAGE  <阶段>  elapsed_s=<秒>
```

它们用于判断 ROS master、导航/坡道、相机、任务视觉和语音哪个阶段最慢。没有看到
`MONITORED_FINALS_READY` 时不得说订单，也不得把某一个节点单独上线当成整链 READY。

### 6.3 国赛发车前最后检查

另开只读终端：

```bash
cd /home/ucar/qhh
./scripts/mission/status_full_chain_monitor.sh --follow
```

再次确认仿真双机链：

```bash
if timeout 5 rostopic echo -n 1 /simulation/link_status | \
  /usr/bin/python3 -m src.mission.simulation_protocol --check-link-status-echo; then
  echo SIMULATION_LINK_READY
else
  echo SIMULATION_LINK_NOT_READY
fi
```

只有以下条件全部成立才说一次第 1 章完整订单：

- `MONITORED_FINALS_READY` 已出现；
- 仿真输出 `SIMULATION_LINK_READY`；
- 车辆位于国赛发车点且 AMCL 正确；
- 电池、插头和电池仓固定；
- 坡道、领取区、动态灯、巡线区无人，急停人员就位；
- 没有旧导航目标、第二套相机、第二套巡线或第二套仿真 bridge。

动态灯保持红灯时长期等待属于正常规则状态。相机释放连续 3 次超时、finals2 订阅者失联
超过 2 秒、坡道领取路线超过外层时限或任一关键传感器失效时，代码会进入明确失败/中止，
不得人工从中间状态续跑。

### 6.4 国赛停止、急停与日志目录

正常结束或重新开局：

```bash
cd /home/ucar/qhh
./scripts/mission/stop_monitored_finals_run.sh
```

测试中出现运动风险且仍可用 ROS 时：

```bash
cd /home/ucar/qhh
./scripts/mission/emergency_stop_finals.sh
```

物理危险时优先使用物理急停/断电。正式计时后操作电脑通常会使本轮失效，急停脚本仅用于
安全风险或裁判授权。

国赛日志目录：

```text
/home/ucar/qhh_competition_logs/YYYYmmdd_HHMMSS_finals-full/
```

其中包含坡道状态、领取路线、相机释放、坡道输出开关、动态灯、巡线模式、运动开始时间、
组件日志和源码哈希。事件字段解释继续看第 7 章。

## 7. 日志与快速判断

另开只读终端：

```bash
cd /home/ucar/qhh
./scripts/mission/status_full_chain_monitor.sh --follow
```

不持续跟随：

```bash
./scripts/mission/status_full_chain_monitor.sh
```

每趟日志目录：

```text
/home/ucar/qhh_competition_logs/YYYYmmdd_HHMMSS_score-first/
/home/ucar/qhh_competition_logs/YYYYmmdd_HHMMSS_dynamic-task2/
/home/ucar/qhh_competition_logs/YYYYmmdd_HHMMSS_finals-full/
```

重点文件：

- `timeline.tsv`：人工可读全链时间线；
- `events.jsonl`：原始结构化事件；
- `summary.json`：最后状态和关键回执；
- `startup.log`：冷启动失败原因；
- `component_logs/`：停栈时归档的各节点日志；
- `source_commit.txt/source_status.txt/source_sha256.txt`：本趟实际代码证据。

监控从 ROS master 出现前就已启动等待；终端中的事件名可直接判断：

| 事件 | 应看到的关键内容 |
|---|---|
| `WAKE_SIGNAL` / `ASR_TEXT` / `VOICE_COMMAND_CHECK` | 唤醒、ASR 原文、规范化口令与 `accepted` |
| `QR_DETECTED` / `QR_THREE_ITEMS_LOCKED` | 每次当前命中和最终三个具体货品 |
| `SPARK_REAL_RESULT` / `SPARK_SIM_RESULT` | 两路分类返回的具体货品与类别 |
| `OCR_WAREHOUSE` / `WAREHOUSE_SEARCH_*` | 第二轮厂牌文字、目标、投票、请求和结果 |
| `SIMULATION_*` | link、request、ACK、进度和最终 success |
| `ANNOUNCEMENT_TEXT` / `PLAYBACK_RECEIPT` | 完整逐字播报与播放程序回执 |
| `TRAFFIC_VISION` / `TRAFFIC_GATE` / `LINE_MODE` | 红绿灯视觉、安全放行和巡线模式 |
| `STOP_LINE_ARRIVED` / `LINE_HANDOFF_ARM` / `LINE_END_SIGNAL` | 到达停止线、巡线交接和巡线终点 |
| `TASK5_ANNOUNCEMENT_ARM` / `TASK5_STATUS` | 任务五播报触发和最终完成状态 |
| `FINALS_PICKUP_ROUTE` / `RAMP_STATUS` | 国赛坡道领取路线和坡道控制器状态 |
| `CAMERA_RELEASE` / `RAMP_OUTPUT_ENABLE` | 国赛交通灯到巡线的资源交接 |
| `FINALS_LINE_STATUS` / `FINALS_PICKUP_ROUTE_ABORT` | 国赛巡线状态和领取路线中止证据 |

QR、OCR 和交通灯采用“语义变化立即记录、稳定结果限频记录”，既保留每次新命中又避免
高频帧淹没任务状态。日志可证明软件识别、生成文字和播放程序回执；仍不能单独证明现场
扬声器音量，现场应同时录像并保留整趟日志目录。

关键状态顺序：

```text
WAIT_ORDER
WAIT_QR
WAIT_CLASSIFICATION
WAIT_SIM_CLASSIFICATION
ANNOUNCING_TASK1
TASK2_SCORE_FIRST_BYPASS / WAIT_DYNAMIC_WAREHOUSE_PARKING
WAIT_SIMULATION_LINK_READY
WAIT_SIMULATION_ACK
WAIT_SIMULATION_RESULT
ANNOUNCING_TASK3
TASK4_READY
任务五完成
```

## 8. 结束、重开和异常恢复

### 8.1 预赛/省赛停止与恢复

巡线或导航出现碰撞风险时，先用物理急停/断开电机保障安全；若车辆仍可由 ROS 控制且处于
测试阶段，运行专用急停：

```bash
cd /home/ucar/qhh
./scripts/mission/emergency_stop_preliminary.sh
```

必须看到 `EMERGENCY_STOP_COMPLETE`。该脚本先撤销巡线和任务五授权、取消 move_base、只按
PID 所有权停止 `visual_navigation_3`，再连续发布零速；不会 `pkill` 其他队友节点。正式计时
后操作电脑通常会使本轮失效，只有安全风险或裁判明确授权时使用，不能把它当作比赛中的
人工续跑入口。

正常结束或主动中止：

```bash
cd /home/ucar/qhh
./scripts/mission/stop_preliminary_field_stack.sh competition
```

一趟结束后重开：

1. 完整停止；
2. 车辆放回 P 点并对正；
3. 核对 AMCL 和仿真 link status；
4. 重新执行第 5 章对应的预赛命令；
5. 不在旧状态机上再次注入订单。

仅 SSH 断线、车辆和节点仍正常时不要冷启动：

```bash
ssh ucar@<小车IP>
cd /home/ucar/qhh
./scripts/mission/status_preliminary_field_stack.sh competition
./scripts/mission/status_full_chain_monitor.sh
```

### 8.2 全国总决赛停止与恢复

国赛运动风险且仍可通过 ROS 停车时：

```bash
cd /home/ucar/qhh
./scripts/mission/emergency_stop_finals.sh
```

国赛正常结束或重新开局：

```bash
cd /home/ucar/qhh
./scripts/mission/stop_monitored_finals_run.sh
```

仅 SSH 断线且车辆、坡道控制器和节点均正常时，不要重复冷启动：

```bash
ssh ucar@<小车IP>
cd /home/ucar/qhh
./scripts/mission/status_finals_field_stack.sh competition
./scripts/mission/status_full_chain_monitor.sh
```

若总状态机已经 `ABORTED`、领取路线出现 `FAILED/ABORTING`、相机交接达到 terminal failure，
或 finals2 已退出，必须完整停止、车辆放回国赛发车点、核对 AMCL 后按第 6 章开新趟次；
不得在旧状态机上重新发订单。

### 8.3 两阶段共用的换网与断电恢复

换网络后：

1. 仿真电脑与小车重新入同一网段；
2. 更新仿真端 vehicle host；
3. SSH 到新 IP；
4. 完整停止旧趟次；
5. 重新执行第 4 节和对应发车命令。

运行中断电：

1. 不继续旧趟次；
2. 断电检查电池仓并重新上电；
3. 获取新 IP；
4. 重新连接，执行停止命令清理残留 PID；
5. 车辆放回 P 点、重设并核对 AMCL；
6. 仿真端重新确认 ready；
7. 预赛按第 5 章对应策略、国赛按第 6 章开全新趟次。

禁止手工删除 PID、叠加第二套导航、`pkill -9`、从中间状态猜测恢复。

## 9. 尚未实车验收

以下能力只有源码与离线测试，不能写成正式通过：

- 随机厂区 1～7 搜索完整实车成功率；
- 三轮完整进入停车区；
- 任务二实体与仿真搜索共享 120 秒的实车全链；
- 搜索降级后任务三至五的正式计分结果；
- 决赛坡道、前轮停止线观察器和决赛挡板巡线。

另有一项必须在赛前向裁判/技术支持确认：当前搜索控制器把 1～7 作为生产区边界的稀疏
视觉观察目标，并继续使用官方 move_base 与实时雷达闭环。规则允许少量有明确任务语义的
分段 goal，但禁止用高密度固定点编码预设轨迹；在取得“这 7 个观察目标属于允许边界”的
明确口径前，不得把该实现写成已通过合规审查，更不得增加密集点或预存候选路线。

预赛与国赛现在共用本手册的合同、仿真和日志说明，但发车入口保持严格分章：预赛只用第 5
章，国赛只用第 6 章。任何未取得同版本实车日志的能力仍保持“未验收”，不能因为命令已经
写入 Full Chain 就宣称规则通过。
