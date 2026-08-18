# Gazebo 仿真队友代码接入与全流程联调交接单

> 编写日期：2026-08-18
> 接收人：Gazebo 仿真任务开发队友
> 目标环境：原生 Ubuntu 20.04 + ROS Noetic + Gazebo Classic + RViz + 腾讯会议
> 仓库基线：`main`，编写时 `HEAD=4ee5160`
> 当前结论：双机通信接口已进入仓库；你电脑上已经完成的“取货、运输、放置”仿真主体尚未合入。请在保留现有接口的前提下接入你的代码，并完成原生 Ubuntu 与实体车的全流程联调。

## 1. 你需要完成的最终交付

请把你电脑上已经能完成完整 Gazebo 任务的源码合入本仓库，使下面整条链路无需人工点击即可运行：

```text
赛前启动 Gazebo、RViz、仿真任务节点和 bridge
→ 仿真电脑主动连接实体车 TCP 24580
→ 实体车完成任务 2 播报
→ 实体车到仿真货品对应厂区并规范停泊
→ 实体车自动发送唯一仿真请求
→ bridge 自动调用 /sim_task/execute
→ 虚拟机器人识别并抓取正确物块
→ 使用官方全局规划器和实时雷达闭环运输
→ 机械臂把物块放入正确仓库
→ Action 返回 success=true、completed_stage=20
→ bridge 返回真实完成结果
→ 实体车播报任务 3 规定文本
→ 播报完成后实体车自动进入任务 4
```

你的交付不是“单机 Gazebo 可以跑”，而是：

- 完整仿真代码已通过 Git 提交进入仓库；
- Ubuntu 工作空间可从干净源码重新编译；
- 三类货品都能走到 `TASK_COMPLETED=20`；
- 与实体车完成真实局域网通信和自动触发；
- 成功、失败、断线和重复请求行为均符合本交接单；
- 提供可审查源码、测试记录、日志和录屏。

## 2. 正式规则红线

以下要求来自当前基础规则手册和当届秩序册，接入代码时不能弱化：

1. 实体车必须在任务 2 播报结束后，到仿真货品对应厂区规范停泊，才可发送仿真就绪信号。
2. 实体车必须等待仿真系统真实完成反馈；仿真失败、超时或掉线不能伪装成完成。
3. 只有物块被正确抓取并放置到正确仓库，仿真任务才算完成。
4. Gazebo 和 RViz 必须同时启动、全程可见，不能最小化或互相完全遮挡。
5. 仿真电脑必须全程进入指定腾讯会议，并完整、实时共享仿真界面。
6. Gazebo 的 `real_time_factor` 不得超过 `1:1`。
7. 官方 `spawn_cubes.py` 不得修改。
8. 全局路径规划只能使用官方统一提供的全局规划器；不得修改、二次封装、拦截或覆盖其输出路径。
9. 不得硬编码赛道坐标、障碍物坐标、全局关键点或固定行驶轨迹。
10. 局部运动和避障必须以实时激光雷达点云为核心输入。
11. 禁止订阅 `/gazebo/model_states`，也禁止通过其他 Gazebo 真值接口变相读取精准位姿或物体答案。
12. 正式运行中禁止人工遥控、点击 RViz 目标、手工发速度、修改参数或重启流程。
13. 裁判抽检时必须能当场提交未加密、无混淆的完整源码、启动脚本、参数和依赖说明。

队友勘误中提出的 fail-open 方案不采用。失败后仍播报成功会和腾讯会议画面直接矛盾，也不满足“等待仿真完成反馈”的规则要求。当前车端已实现 fail-closed：失败、超时、掉线、身份不匹配或阶段不足时停止本轮自动链路，不误播成功。

## 3. 当前仓库进度

### 3.1 已完成：实体车侧

| 文件 | 当前作用 |
| --- | --- |
| `src/mission/simulation_protocol.py` | 任务 3 请求构建、类别映射、NDJSON 编解码、结果身份校验 |
| `src/mission/simulation_link_node.py` | 实体车 TCP 服务端，监听仿真电脑连接并转发 ROS 消息 |
| `src/mission/factory_mission_core.py` | 校验 session、request、order、类别、阶段 20 和 rehearsal 标记 |
| `src/mission/factory_mission_node.py` | 停稳后等待 ready，发送请求，等待 ACK/结果，播报后继续任务 4 |
| `src/mission/simulation_rehearsal_stub_node.py` | 仅本地排练使用；正式模式不接受假结果 |
| `scripts/mission/start_factory_visual_stack.sh` | competition 模式自动启动真实 `simulation_link_node` |
| `scripts/mission/status_factory_visual_stack.sh` | 只读检查仿真链路和任务节点 |
| `scripts/mission/ready_factory_chain.sh` | 发车前要求 `/simulation/link_status` 为 ready |
| `scripts/deploy_core_to_car.sh` | 将已审查源码部署到实体车，不自动发车 |

车端正式话题：

```text
/simulation/mission_request
/simulation/mission_ack
/simulation/mission_progress
/simulation/mission_result
/simulation/link_status
```

### 3.2 已完成：Ubuntu 仿真电脑侧 bridge

bridge 位于：

```text
Gazebo/gazebo_ws_zqy/src/smart_factory_bridge/
```

| 文件 | 当前作用 |
| --- | --- |
| `scripts/vehicle_bridge_node.py` | 接收 TCP 请求，自动调用 `/sim_task/execute` Action |
| `src/smart_factory_bridge/protocol.py` | 请求字段校验、请求指纹和协议错误处理 |
| `src/smart_factory_bridge/tcp_client.py` | TCP 客户端、心跳、断线重连、发送队列 |
| `src/smart_factory_bridge/action_adapter.py` | Action feedback/result 转为 ACK、progress、result |
| `launch/bridge.launch` | bridge 启动入口 |
| `config/bridge.yaml` | 端口、超时、传感器新鲜度、RViz 要求和结果缓存 |
| `test/` | 协议、阶段 20 判定和 socket 往返测试 |

`Gazebo/gazebo_ws_zqy/src/smart_factory_bringup/launch/full_competition.launch` 已经包含：

```text
start_bridge
vehicle_host
vehicle_port
bridge_config
```

### 3.3 尚未完成：当前仓库中的仿真主体

当前仓库不是你勘误中描述的完整版：

- `smart_factory_mission/states.py` 只定义到 `OBJECT_GRASPED=14`；
- `smart_factory_mission/mission_server.py` 抓取后立即返回阶段 14 成功；
- `smart_factory_mission/config/mission.yaml` 仍为 `pipeline_stop_after: OBJECT_GRASPED`；
- 因此 bridge 会如实把当前 Action 判为失败，实体车不会播报任务 3 成功。

你需要把自己电脑上已经完成的阶段 15、16、17、18、20 及其依赖合入这里。

### 3.4 已完成的本地测试

接口实现时已经得到：

```text
根仓 pytest：200 passed, 1 skipped
bridge 协议与真实 socket 单测：3 passed
Python py_compile：通过
Shell bash -n：通过
JSON/XML 静态检查：通过
```

这些结果只证明接口代码的桌面测试，不代表 ROS Noetic 编译、真实 Gazebo 动作和双机整链已验收。

## 4. 合并时必须保留的接口合同

### 4.1 Action 名称和 Goal

Action 名称必须保持：

```text
/sim_task/execute
```

Goal 保持：

```text
string task_id
uint8 target_class
```

类别编号保持：

```text
0 = FOOD
1 = DAILY
2 = ELECTRONICS
```

仿真任务只能根据 `target_class` 和实时感知选择货品与仓库，不能让实体车发送 Gazebo 坐标或固定路径点。

### 4.2 阶段编号

请采用你勘误中的阶段序列：

```text
0-14  现有接单、导航、识别与抓取流程
15    GET_DELIVERY_GOAL
16    NAVIGATE_TO_DELIVERY
17    ARRIVED_DELIVERY
18    RELEASE_OBJECT
20    TASK_COMPLETED
250   TASK_FAILED
```

阶段 19 可以不存在，放置验证可折叠在阶段 18 内。但在物块实际放置完成前，绝不能提前发布 20。

`ExecuteTask.action` 已经有 15、16、17、18、20、250 的 feedback 常量，不要删改字段或重排数值。若你的本机 Action 文件不同，请以仓库版本为接口基线调整任务代码。

### 4.3 成功条件

bridge 只在以下条件全部满足时发出 completed：

```text
Action terminal status == SUCCEEDED
result.success == true
result.completed_stage == 20
```

实体车还会继续校验：

```text
state == completed
request_session_id、request_id、order_id 与本轮一致
target_class 与请求一致
rehearsal == false
```

阶段 14、Action aborted/preempted、`success=false`、阶段 250、旧请求结果或排练结果都不能放行。

### 4.4 TCP 协议

- 实体车是 TCP 服务端，默认监听 `0.0.0.0:24580`。
- Ubuntu bridge 是 TCP 客户端，主动连接实体车局域网 IP。
- 两端各自使用独立 ROS Master，不配置 ROS1 多机 Master。
- 一条消息为一行 UTF-8 JSON，即 NDJSON，`schema_version=1`。
- 同一个 `request_id` 和相同内容只能执行一次；重发时返回缓存状态或结果。
- 同一个 `request_id` 对应不同内容时必须拒绝为 `request_conflict`。
- 同时只允许一个活动任务。

请求示意：

```json
{
  "schema_version": 1,
  "message_type": "request",
  "session_id": "car-...",
  "request_id": "order-...:simulation:...",
  "order_id": "order-...",
  "task_type": "gazebo_pick_and_place",
  "target_class": 2,
  "simulation_product": "电脑",
  "simulation_category": "electronics",
  "simulation_warehouse": "电子产品生产车间",
  "physical_station": "C",
  "issued_at": 0.0,
  "timestamp": 0.0
}
```

最终结果示意：

```json
{
  "schema_version": 1,
  "message_type": "result",
  "session_id": "sim-...",
  "request_session_id": "car-...",
  "request_id": "order-...:simulation:...",
  "order_id": "order-...",
  "target_class": 2,
  "state": "completed",
  "success": true,
  "completed_stage": 20,
  "completed_stage_name": "TASK_COMPLETED",
  "rehearsal": false,
  "error_code": 0,
  "message": "...",
  "timestamp": 0.0
}
```

通常不需要修改 bridge 的 TCP 协议。优先让你的任务代码适配现有 Action；只有发现不可兼容的明确字段问题时，先提交接口变更说明和兼容方案再改 bridge。

## 5. 你需要合入哪些代码

### 5.1 必须提交

请从你已经跑通的仿真电脑中找出并提交：

- `smart_factory_mission` 中完整的阶段常量、状态转换和任务服务器；
- 从抓取结束到运输、抵达、释放和完成验证的实现；
- delivery 目标获取逻辑及其配置；
- 机械臂放置动作和放置结果判定；
- 运输阶段依赖的导航、感知或 manipulation 源码修改；
- 对应的 `launch`、`yaml`、`package.xml`、`CMakeLists.txt`；
- 阶段 15-20、失败、抢占和重复任务测试；
- 新增依赖和安装说明。

### 5.2 不要提交

不要提交这些机器生成文件：

```text
Gazebo/gazebo_ws_zqy/build/
Gazebo/gazebo_ws_zqy/devel/
Gazebo/gazebo_ws_zqy/install/
log/
*.bag
__pycache__/
*.pyc
.vscode/
```

不能用 `build` 或 `devel` 覆盖源码问题。验收必须能从 `src/` 干净重建。

### 5.3 禁止覆盖或回退

合并时必须保留：

- 整个 `smart_factory_bridge` 当前实现；
- `full_competition.launch` 中的 bridge 参数和 include；
- `/sim_task/execute` Action 名和当前 Goal/Result/Feedback 字段；
- 阶段 15-18、20、250 的数值；
- bridge 对阶段 20 的严格成功判定；
- 实体车的 session/request/order 去重与 fail-closed 逻辑；
- 原版 `spawn_cubes.py`。

如果你的完整任务代码改过这些文件，请逐块合并，不要直接用整个旧文件夹覆盖当前 `src/`。

## 6. 建议的 Git 接入流程

请先保留你电脑上能运行的版本，不要直接在唯一副本上操作。

### 6.1 在你的仿真电脑记录现状

```bash
cd ~/iflytek-smart-car
git status --short
git rev-parse HEAD
git diff -- Gazebo/gazebo_ws_zqy/src > ~/gazebo_local_changes.patch
```

如果你的完整代码还没有 Git 提交，先在自己的备份分支提交一次。不要把 `build/`、`devel/` 和 bag 加进去。

### 6.2 从最新主线创建接入分支

推荐在一个干净 clone 中操作：

```bash
cd ~
git clone <项目 GitHub 地址> iflytek-smart-car-integration
cd ~/iflytek-smart-car-integration
git pull --ff-only origin main
git switch -c feature/gazebo-delivery-pipeline
```

然后只复制或 cherry-pick 你的仿真源码改动。复制后先检查：

```bash
git status --short
git diff --stat
git diff -- Gazebo/gazebo_ws_zqy/src/smart_factory_bridge
git diff -- Gazebo/gazebo_ws_zqy/src/smart_factory_bringup/launch/full_competition.launch
git diff -- Gazebo/gazebo_ws_zqy/src/smart_factory_interfaces/action/ExecuteTask.action
```

后三项若出现大面积回退或删除，先停止合并并逐块处理。

### 6.3 建议的文件级修改

1. 在 `smart_factory_mission/src/smart_factory_mission/states.py` 增加阶段 15、16、17、18、20，并写入 `NAMES`。
2. 在 `mission_server.py` 中让 `_continue_after_arrival()` 在抓取后继续执行 delivery pipeline，不再调用阶段 14 的 `_finish_success()`。
3. 将真实运输、抵达、释放、验证逻辑接入任务服务器，并持续发布 Action feedback。
4. 只有物块正确放置后才调用 `_finish_success(..., TASK_COMPLETED, ...)`。
5. 将 `smart_factory_mission/config/mission.yaml` 的正式终点改为 `TASK_COMPLETED`，并让服务器接受该配置值。
6. 合入你的 delivery goal、导航和机械臂配置，但不得引入固定赛道路径或绕过实时感知。
7. 保持 `full_competition.launch` 只启动一套 Gazebo、导航、感知、任务服务器和 bridge，避免重复节点争用话题。
8. 增加测试，至少覆盖成功到 20、释放失败、导航失败、Action 抢占和重复 `task_id`。

### 6.4 提交

```bash
git add Gazebo/gazebo_ws_zqy/src
git status --short
git diff --cached --stat
git commit -m "Complete Gazebo delivery and release pipeline"
git push -u origin feature/gazebo-delivery-pipeline
```

把分支名、提交哈希和修改文件清单发回来审查，不要只发送压缩包或 `build/devel`。

## 7. 原生 Ubuntu 编译与静态检查

### 7.1 安装依赖并编译

```bash
cd ~/iflytek-smart-car-integration/Gazebo/gazebo_ws_zqy
source /opt/ros/noetic/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
rm -rf build devel
catkin_make
source devel/setup.bash
```

删除 `build/devel` 是为了证明源码能够干净重建，只在你确认当前目录确实是该 catkin 工作空间后执行。

### 7.2 红线扫描

```bash
cd ~/iflytek-smart-car-integration
rg -n '/gazebo/model_states|model_states|set_model_state|set_model_configuration' \
  Gazebo/gazebo_ws_zqy/src
rg -n 'cmd_vel|MoveBaseGoal|SimpleActionClient.*move_base' \
  Gazebo/gazebo_ws_zqy/src
git diff origin/main...HEAD -- \
  Gazebo/gazebo_ws_zqy/src | less
```

扫描命中不一定等于违规，但每个命中都必须解释其用途。尤其要确认没有真值定位、固定全局路径、人工速度入口和第二套全局规划器。

同时确认官方脚本没有变化：

```bash
git diff origin/main...HEAD -- '**/spawn_cubes.py'
```

正常结果应为空。

## 8. 分阶段联调

每一阶段通过后再进入下一阶段。实验用手工触发只能用于受控开发测试，并在正式验收前移除或关闭；比赛模式不得依赖手工命令。

### 8.1 A：仿真 Action 单机测试

启动完整仿真，但先不开 bridge：

```bash
cd ~/iflytek-smart-car-integration/Gazebo/gazebo_ws_zqy
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch smart_factory_bringup full_competition.launch \
  start_bridge:=false
```

检查：

```bash
rostopic hz /clock
rostopic hz /scan
rostopic hz /amcl_pose
rostopic echo /sim_task/state
```

使用仓库已有测试客户端或你已有的 Action 测试工具依次发送类别 0、1、2。必须观察到：

```text
14 OBJECT_GRASPED
15 GET_DELIVERY_GOAL
16 NAVIGATE_TO_DELIVERY
17 ARRIVED_DELIVERY
18 RELEASE_OBJECT
20 TASK_COMPLETED
```

同时实际观察物块是否进入正确仓库，不能只看状态数字。

### 8.2 B：双机只读连通

实体车先启动 competition 栈：

```bash
cd /home/ucar/qhh
export SIMULATION_BIND_HOST=0.0.0.0
export SIMULATION_PORT=24580
./scripts/mission/start_factory_visual_stack.sh \
  --mode competition --arm-competition-stack
```

Ubuntu 查看实体车 IP 是否可达：

```bash
ping -c 4 <实体车局域网IP>
nc -vz <实体车局域网IP> 24580
```

Ubuntu 开启 bridge 后，在实体车查看：

```bash
rostopic echo /simulation/link_status
```

必须看到 `connected=true`、`ready=true`。此阶段不发车、不发布速度，只验证心跳和就绪条件。

### 8.3 C：真实请求与完整 Action

Ubuntu 启动命令：

```bash
cd ~/iflytek-smart-car-integration/Gazebo/gazebo_ws_zqy
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch smart_factory_bringup full_competition.launch \
  start_bridge:=true \
  vehicle_host:=<实体车局域网IP> \
  vehicle_port:=24580
```

实体车只读预检：

```bash
cd /home/ucar/qhh
./scripts/mission/status_factory_visual_stack.sh competition
./scripts/mission/ready_factory_chain.sh competition
```

在安全架空轮或封闭场地条件下运行到任务 3 触发点，记录：

- `/simulation/mission_request`；
- `/simulation/mission_ack`；
- `/simulation/mission_progress`；
- `/simulation/mission_result`；
- `/mission/status`；
- Ubuntu 的 Action feedback/result；
- Gazebo 画面、RViz 画面和实体车播报。

验收点是：阶段 20 返回后才播报；播报真正结束后才进入任务 4。

### 8.4 D：异常测试

正式冻结前至少验证：

| 场景 | 预期结果 |
| --- | --- |
| Action 正常完成阶段 20 | 实体车播报任务 3，播完进入任务 4 |
| Action 在阶段 14 结束 | bridge 返回 failed，实体车不误播、不继续 |
| 放置失败或 Action aborted | 返回真实失败，实体车停止本轮链路 |
| 请求后短暂断网再恢复 | 自动重连，同一请求不重复执行 |
| 同一请求重复到达 | 返回缓存状态/结果，只执行一次 |
| 同 ID 不同内容 | `request_conflict`，不执行 |
| 旧 session 或旧 order 结果到达 | 实体车拒绝 |
| Gazebo 暂停、雷达/定位过期或 RViz 未启动 | heartbeat `ready=false`，发车预检阻塞 |
| bridge 或任务节点退出 | 实体车超时并 fail-closed |

## 9. 比赛启动顺序

### 9.1 Ubuntu 仿真电脑

1. 关闭会改变默认路由的 VPN。
2. 打开腾讯会议，进入指定会议并共享整个桌面。
3. 调整窗口，让 Gazebo、RViz 和 Gazebo 底部 RTF 同时可见。
4. 按官方流程重新运行原版 `spawn_cubes.py`。
5. 启动 `full_competition.launch`，并传入本轮实体车 IP。
6. 检查 `/clock`、`/scan`、`/amcl_pose`、Action server、RViz 和 bridge。
7. 所有节点保持等待，不在实体车到点后临时打开 Gazebo。

### 9.2 实体车

```bash
cd /home/ucar/qhh
export SIMULATION_BIND_HOST=0.0.0.0
export SIMULATION_PORT=24580
export SIMULATION_LINK_WAIT=15
export SIMULATION_ACK_WAIT=8
export SIMULATION_TASK_TIMEOUT=300

./scripts/mission/start_factory_visual_stack.sh \
  --mode competition --arm-competition-stack
./scripts/mission/status_factory_visual_stack.sh competition
./scripts/mission/ready_factory_chain.sh competition
```

只有出现 `STACK_OK`、`CHAIN_READY`，且人工确认车辆、电量、定位、场地和急停条件后才能准备发车。上述脚本本身不会发车。

## 10. 你需要回传的材料

完成后请一次性提供：

- Git 分支名和最终提交哈希；
- `git diff --stat origin/main...HEAD`；
- 具体修改文件清单；
- `catkin_make` 从干净工作空间构建成功的末尾日志；
- 单元测试命令和完整通过结果；
- 三类货品各至少一次阶段 15-20 的 Action 日志；
- 至少一次真实 TCP request/ACK/progress/result 对应日志；
- Gazebo 正确抓取、运输、放置的录屏；
- 实体车收到阶段 20 后播报并进入任务 4 的整链录屏；
- 失败、断线和重复请求测试记录；
- Ubuntu 版本、ROS 版本、依赖版本和启动命令；
- 所有仍未通过的项目及复现步骤。

不要只回复“已跑通”。没有提交、日志和画面证据的能力仍按“尚未验收”处理。

## 11. 完成定义

只有以下项目全部满足，才能把任务标记为“全流程完成”：

- [ ] 完整 delivery pipeline 已合入 Git，不再停在阶段 14。
- [ ] 原生 Ubuntu 20.04 从干净 `src/` 执行 `catkin_make` 成功。
- [ ] FOOD、DAILY、ELECTRONICS 三类均实际放置正确并返回阶段 20。
- [ ] 原版 `spawn_cubes.py` 多轮运行后仍可完成任务。
- [ ] 官方全局规划器、实时雷达闭环和禁止真值/硬编码要求通过代码审查。
- [ ] bridge 可自动连接实体车，ready 状态真实反映 Gazebo/RViz/传感器/Action。
- [ ] 请求、ACK、progress 和 result 的身份字段完全一致。
- [ ] 重复请求不会重复执行，断线恢复不会串入旧结果。
- [ ] 所有失败路径均 fail-closed，不出现虚假成功播报。
- [ ] 阶段 20 后实体车播报内容正确，播报完成后自动进入任务 4。
- [ ] Gazebo 与 RViz 在腾讯会议共享下全程可见，RTF 全程不超过 1。
- [ ] 完整源码、启动脚本、参数、依赖和日志可以当场提交审查。

## 12. 相关文档

接入前请同时阅读：

- `docs/Gazebo仿真双机通信接口开发任务书.md`：接口实现原理和两台设备使用说明；
- `docs/Gazebo仿真双机通信接口开发任务书_本机偏差勘误.md`：你电脑与旧任务书的差异记录，其中 fail-open 条目已被本方案否决；
- `docs/第二十一届讯飞智慧工厂竞赛规则总手册.md`：基础规则与仿真红线；
- `docs/第二十一届讯飞智慧工厂全国总决赛规则手册.md`：全国总决赛叠加要求；
- `Gazebo/虚拟仿真总方案.md`：Ubuntu、ROS、Gazebo 和官方工作空间的使用说明；
- `docs/real_competition_field_debug_log.md`：真实双机和全链测试必须追加的证据日志。

这份交接单是本次合并和联调的执行基线。若你的本机实现与本文仍有差异，请先列出“文件、接口、行为、原因、兼容方式”五项，再开始改动，避免用旧工作空间直接覆盖新接口。
