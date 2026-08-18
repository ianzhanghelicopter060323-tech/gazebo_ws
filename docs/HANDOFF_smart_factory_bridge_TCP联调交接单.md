# smart_factory_bridge 双机 TCP 通信接入与联调交接单

> 编写日期：2026-08-18（v2.2：队友核查修复 + 合规结论/真值接口处置定案，见第 6、7 节）
> 接收人：Gazebo 仿真任务开发队友 / 实体车联调队友
> 目标环境：原生 Ubuntu 20.04 + ROS Noetic + Gazebo Classic + RViz
> 当前分支：`TEB_test_bridge_fix`（基线 `TEB_test@39d750c`）
> 协议基线：`docs/Gazebo仿真双机通信接口开发任务书.md` 第 6 节
> 　　＋ `docs/Gazebo仿真双机通信接口开发任务书_本机偏差勘误.md`（阶段 15–18 delivery 命名、无阶段 19、progress 格式冻结）
> 当前结论：**仿真侧 bridge 包已实现完毕，51 项桌面单测全部通过（含 ResourceWarning 零泄漏）；尚未与实体车做真实局域网联调。** 下文给出使用方法和联调测试清单。

## 1. 当前进度

### 1.1 已交付内容

`src/smart_factory_bridge/` 包（仿真电脑侧双机通信桥）：

```text
scripts/vehicle_bridge_node.py         # ROS 节点：NDJSON 请求 ↔ /sim_task/execute Action ↔ 就绪检查
src/smart_factory_bridge/protocol.py   # NDJSON 编解码、严格校验、请求指纹、ack/progress/result/error 构造
src/smart_factory_bridge/tcp_client.py # 心跳、退化/断连检测、指数退避重连、离线发送队列、drop_pending
src/smart_factory_bridge/action_adapter.py  # Action feedback/result → progress/result 消息映射
src/smart_factory_bridge/readiness.py  # ROS-free：墙钟新鲜度（FreshnessState）、laser 有效性、heartbeat 构造
config/bridge.yaml                     # 端口、超时、传感器新鲜度、RViz 检查、结果缓存等配置
launch/bridge.launch                   # 独立启动入口
test/                                  # 桌面单测（51 项，不依赖 ROS Master）
tools/mock_vehicle_server.py           # mock 车端 TCP 服务端（纯标准库，本地联调用）
setup.py / CMakeLists.txt / package.xml / README.md
```

同时修改了 `src/smart_factory_bringup/launch/full_competition.launch`：新增
`start_bridge`、`vehicle_host`、`vehicle_port`、`bridge_config` 四个参数，`start_bridge:=true`
时自动拉起 bridge。

### 1.2 角色与职责（协议冻结版）

- **仿真电脑 = TCP 客户端**：主动连接实体车 TCP 服务端（默认端口 **24580**）。
- 通信协议：**NDJSON**（一行一个 UTF-8 JSON，`\n` 结尾，单行上限 64 KiB，`schema_version=1`）。
- 车端发 `request` → 仿真侧回 `ack`（accepted）→ 执行中回 `progress`（阶段 15–18 用
  delivery 命名，无阶段 19）→ 终止时回 `result` 或 `error`。
- 实体车侧需提供 TCP 服务端监听 `0.0.0.0:24580`；双机之间只走 TCP，不涉及 ROS Master 互通。

### 1.3 已实现的协议行为（红线不弱化）

| 行为 | 实现 |
|---|---|
| 心跳与就绪 | 每 1 s 发 heartbeat；`ready` 由 Gazebo、RViz、Action server、定位、`laser_ready`、传感器新鲜度和 `busy` 共同决定，任一不满足即 `ready=false`（fail-closed，车端据此阻塞发车）；**`gazebo_ready` 要求 /clock 活跃（墙钟 1 s 内）**——暂停时 /clock 主题仍在但停发，1 s 内 `gazebo_ready=false`；`/scan` 同步过期 → `laser_ready=false`；整体 `ready=false` |
| laser_ready | `"laser_ready"` 定义为**最近收到新鲜且基本有效的 /scan**（墙钟 2 s 内 + 非空 ranges、range_min/range_max 合法、无 NaN），**不是**"是否检测到障碍物"；与 `"sensors_ready"` 同时保留（车端忽略未知字段，向后兼容） |
| 链路退化/重连 | 3 s 无数据 → degraded；10 s → 断开并按 0.5/1/2/4/5 s 指数退避重连（全部为墙钟计时） |
| 去重 | 同一 `request_id` + 相同内容重复到达：不重复执行，从缓存补发 ack/progress/result（车端断线重连后重发 pending_request，由本机制应答） |
| 冲突 | 同一 `request_id` 不同内容：`request_conflict`（ERR_REQUEST_CONFLICT） |
| 忙碌 | 执行期间新请求：`busy`（ERR_BUSY） |
| 成功判定 | **只有 `success=true && completed_stage=20` 才发送 `completed`**；阶段 14、aborted、preempted、任务超时一律如实发送 `failed`，绝不伪装成功（fail-closed 是桥的职责，fail-open 是车端职责） |
| 断线恢复 | **无条件下发**：重连上升沿只补发一条心跳；断线期间产生的 result 经离线队列**恰好补发一次**；车端重发 pending_request 时由去重缓存应答；若同一 result 同时存在于离线队列与缓存，先移除队列副本再重放（`drop_pending`），保证同一结果不因两种机制发送两遍；无对应请求绝不推送上一局残留结果 |
| 墙钟超时 | 新鲜度（/scan、/amcl_pose、/clock）与 Action 总超时（默认 300 s）一律用 `time.monotonic()` 判定；**任务等待用 done callback + `threading.Event.wait()`（纯墙钟轮询）**——`wait_for_result(rospy.Duration)` 的剩余时间由仿真时钟计算，暂停时会永久阻塞导致墙钟超时无法执行，故弃用；**Action server 连接探测同样墙钟化**：弃用 `wait_for_server(Duration)`（同一暂停卡死问题），改为轮询 `is_server_connected()`（0.5 s 截止、50 ms 步长）；ROS 消息 `header.stamp` 由 ROS 自动填充，不受影响；Gazebo 暂停中已开始的任务在墙钟超时后返回 `failed`（fail-closed，车端可接受） |
| 健壮性 | 所有异常只记录不退出；socket 全部确定性关闭（ResourceWarning 零泄漏）；`/simulation/bridge_status` 发布本机状态 JSON（调试用，车端不读取） |

### 1.4 约定与红线（代码层面已保证）

- **bridge 与 mission 无任何 Gazebo 真值接口**（无 `/gazebo/model_states`、
  `get_model_state`、`set_model_state`）。
- 全仓库两处 Gazebo 真值接口的处置（2026-08-18 已定）：
  ① `car3/scripts/grasp_attach.py`——官方 car3 包自带的抓取随动机制，使用
  `/gazebo/get_link_state`、`/gazebo/get_model_state`、`/gazebo/get_world_properties`、
  `/gazebo/set_model_state`，由 `car3/launch/gazebo.launch:34` 启动；属官方包自带
  仿真机制，**保持原样，不做改动**；② `smart_factory_tests/scripts/monitor_navigation_safety.py`
  订阅 `/gazebo/model_states`——测试监控脚本，**正式比赛前从仓库移除**（当前保留
  供联调排查使用）。
- 无第二个全局规划器（使用官方全局规划器）。路线合规结论（2026-08-18 确认）：
  **规则允许设定中途导航点**——固定坐标（仓库口/准备位姿/拟合点）与动态中间
  goal 属中途导航点范畴，合规；`/move_base/make_plan` 仅用于调用官方全局规划器
  筛选候选路线，无二次规划、无干预输出。四类用法**零代码修改、零调参**，
  明细见 `docs/路线合规使用清单.md`。
- 无硬编码 IP：`vehicle_host` 只能来自 launch 参数或环境变量 `SIMULATION_VEHICLE_HOST`。
- 无阶段 19；阶段 15–18 命名遵守偏差勘误文档。

## 2. 使用方法

### 2.1 正式入口（推荐）

```bash
# 工作空间编译
cd /home/ianzhang/gazebo_ws && catkin_make

# 全流程启动，末尾带 bridge
roslaunch smart_factory_bringup full_competition.launch \
  start_bridge:=true \
  vehicle_host:=<实体车局域网IP> \
  vehicle_port:=24580
```

`vehicle_host` 也可改用环境变量：

```bash
export SIMULATION_VEHICLE_HOST=<实体车局域网IP>
```

### 2.2 独立启动 bridge（调试用）

```bash
roslaunch smart_factory_bridge bridge.launch \
  vehicle_host:=<实体车局域网IP> vehicle_port:=24580
```

### 2.3 配置（config/bridge.yaml）

| 配置项 | 默认值 | 说明 |
|---|---|---|
| `vehicle_port` | 24580 | 车端 TCP 端口 |
| `heartbeat_interval` | 1.0 s | 心跳周期 |
| `degraded_after` / `disconnect_after` | 3.0 / 10.0 s | 退化 / 强制断开阈值（墙钟） |
| `reconnect_backoff` | [0.5, 1, 2, 4, 5] s | 指数退避序列（封顶） |
| `max_line_bytes` | 65536 | 单行消息上限 |
| `sensor_topics` | /scan、/amcl_pose | 新鲜度探测主题（**单调墙钟**判定，Gazebo 暂停数秒后自动失效）；`scan` 走 `LaserFreshness`（新鲜 + 结构有效 → laser_ready），`amcl` 驱动 localization_ready |
| `gazebo` | /clock 存在 + /gazebo/get_physics_properties + **/clock 墙钟新鲜度（clock_max_age 1 s）** | 就绪判定；暂停 1 s 内 `gazebo_ready=false` |
| `rviz` | mode: process（pgrep 进程探针） | RViz 就绪判定 |
| `action` | /sim_task/execute，wait 15 s，任务超时 300 s | Action 配置；**总超时用墙钟判定**（暂停中也会超时返回 failed） |
| `result_cache.max_entries` | 64 | 结果缓存上限（去重/重放） |

### 2.4 桌面单测（不依赖 ROS Master）

```bash
cd /home/ianzhang/gazebo_ws/src/smart_factory_bridge
python3 -W error::ResourceWarning -m unittest discover -s test
# 期望输出：Ran 51 tests ... OK（ResourceWarning 视为错误，socket 零泄漏）
```

### 2.5 无实体车本地联调（mock 车端）

`tools/mock_vehicle_server.py` 扮演车端 TCP 服务端（纯标准库，无需 ROS），
用于 heartbeat 示例捕获、暂停就绪失效观察、断线/重连/离线队列补发测试：

```bash
python3 tools/mock_vehicle_server.py --timeout 30          # 观察心跳与就绪
python3 tools/mock_vehicle_server.py --send-request        # 发一个 food 请求触发 Action
python3 tools/mock_vehicle_server.py --crash-after 5 --crash-duration 3  # 模拟车端掉线
```

## 3. 本机（仿真侧）已进行的测试

### 3.1 单测覆盖（51 项，全部通过；`-W error::ResourceWarning` 运行，socket 零泄漏）

| 模块 | 数量 | 覆盖点 |
|---|---|---|
| `test_protocol.py` | 25 | 编解码往返；超长/非法 JSON/非对象/坏 schema/未知类型/缺 session 拒绝；request 校验（缺字段/类型错误/未知类别/类别与 target_class 不一致/未知任务类型）；请求指纹（同内容同指纹——timestamp/issued_at 不计入；不同内容不同指纹）；ack/progress/result/error 构造均回显 request_session_id/request_id/order_id/target_class；result 仅在 `success && stage==20` 时为 completed；无 request 的 error 构造 |
| `test_action_adapter.py` | 10 | ack；feedback→progress 映射（阶段 16→NAVIGATE_TO_DELIVERY，15–18 delivery 命名）；SUCCEEDED+stage 20→completed；**SUCCEEDED+stage 14 必须为 failed**；success=false 即使 stage 20 也为 failed；aborted 透传 error_code、无 error_code 用 255；preempted→8；lost→failed；身份字段恒回显；rehearsal 恒为 false |
| `test_tcp_client.py` | 6 | 真实 socket（127.0.0.1 监听端口）心跳往返与 ack 接收；服务端主动断开→客户端上报 disconnected；**断线重连后离线队列中的 result 补发成功**；**重连本身不补发任何残留结果（仅心跳）**；**`drop_pending` 移除队列副本后不再被 flush（同一结果不会双发）**；超长行丢弃且下一行正常处理 |
| `test_readiness.py` | 10 | 墙钟新鲜度（max_age 内新鲜、过期失效、从未标记不新鲜、多线程安全）；**Gazebo 暂停模拟（停止投喂 → 墙钟推进 → 新鲜度失效，等效 ready=false）**；laser 结构有效性（空 ranges/负 range_min/range_min≥range_max/NaN 一律 fail-closed；全 inf 无回波仍有效——laser_ready ≠ 检测到障碍）；heartbeat 含 `laser_ready`+`sensors_ready`、laser 或 busy 任一不满足即 `ready=false`、heartbeat 可编码为单行 NDJSON（示例见下）；**回归防护：节点源码禁止出现 `wait_for_result(`、`send_goal_and_wait(`、`wait_for_server(`、`rospy.sleep(` 调用点，且必须使用 done_cb + `goal_done.wait` 与 `is_server_connected()` 轮询** |

> heartbeat 示例（测试运行输出，字段为就绪全真 + 空闲）：
> `{"action_server_ready": true, "busy": false, "gazebo_ready": true, "laser_ready": true, "localization_ready": true, "message_type": "heartbeat", "ready": true, "rviz_ready": true, "schema_version": 1, "sensors_ready": true, "session_id": "sim-test", "timestamp": ...}`

### 3.2 静态与红线检查（已执行，全部通过）

- `py_compile` 全部 Python 文件（src、scripts、tools、test）无语法错误。
- `bridge.launch`、`full_competition.launch` XML 解析通过。
- 红线扫描：无 `/gazebo/model_states`；生产代码无硬编码 IP（仅测试代码使用 127.0.0.1 环回）；无阶段 19；`completed` 仅在 `success && stage==20` 路径产生。
- 墙钟替换审计：节点内 actionlib 已无任何依赖仿真时钟的调用——任务等待为 done callback + `threading.Event.wait()`，Action server 连接探测为墙钟轮询 `is_server_connected()`（0.5 s 截止、50 ms 步长）；新鲜度判定与任务截止时间全部走 `time.monotonic()`，**不再有依赖仿真时钟的阻塞等待**（TCP 层本就为墙钟）；ROS 消息 `header.stamp` 由 ROS 自动填充，不受影响。
- 无 `_last_result` 无条件重发路径（`_on_link_state` 仅补发心跳）；`drop_pending` 保证离线队列与去重缓存不同时持有同一 result。

### 3.3 调试记录（重要，供联调参考）

`test_reconnect_flushes_queued_message` 曾稳定失败，双端插桩定位：

1. **客户端逻辑经插桩验证完全正确**：断线期间 `send()` 入队 → 重连后 `_flush_pending()` 把
   result 作为新连接首行送达服务端（服务端 recv 实际收到 `{"completed_stage": 20, "message_type": "result"...}`）。
2. 失败根因在测试代码自身两处：测试辅助函数 `read_line` 丢弃同一 recv 块中首个换行后的内容；
   handler 用连接首行做 conn1 探测后将其丢弃，而补发的 result 恰是连接首行。
3. 修复测试代码（`LineReader` 保留缓冲区、非首个连接首行也 append），**客户端代码零改动**。

结论：本机端"离线排队 → 重连补发"链路已在真实 TCP 层面验证，联调时若车端收不到补发结果，
请优先排查车端读取逻辑，而非仿真侧发送逻辑。

## 4. 后续与小车端需进行的联调测试

> 前置条件：实体车 TCP 服务端已监听 `0.0.0.0:24580`；两端在同一局域网；协议版本
> `schema_version=1`；车端按冻结协议（任务书 §6 + 偏差勘误）收发。

### 4.1 网络连通性（先做）

- [ ] 仿真电脑 `ping` 通实体车 IP。
- [ ] `nc -vz <实体车IP> 24580`（或 `telnet`）确认端口开放；若不通检查车端防火墙/绑定地址。
- [ ] 仿真电脑启动 bridge 后，车端日志确认收到心跳（每 1 s 一条），且 `ready` 状态符合预期。
- [ ] 断网重插网线：观察重连退避（0.5/1/2/4/5 s）与心跳恢复。

### 4.2 就绪判定（ready 语义）

- [ ] 正常全套启动（Gazebo + RViz + 任务节点 + 定位 + 雷达）：heartbeat `ready=true`，
      且 `laser_ready=true`、`sensors_ready=true`。
- [ ] Gazebo 暂停（`/clock`、`/scan` 停走）：数秒内 `ready` 与 `laser_ready` 变为 `false`，
      车端不得发车；恢复播放后数秒内恢复 `true`。
- [ ] 关掉 RViz（进程探针）：`ready=false`。
- [ ] 关掉 Action server 或 /scan、/amcl_pose 数据：`ready=false`。
- [ ] 任务执行期间 heartbeat `busy=true`（`ready=false`）。
- [ ] 暂停期间任务已开始：墙钟超时（默认 300 s）后返回 `failed`，绝不返回 `completed`。

### 4.3 请求主流程（正常路径）

- [ ] 车端发送合法 `request` → 仿真侧回 `ack(state=accepted)`，且身份字段（request_session_id、
      request_id、order_id、target_class）完全回显。
- [ ] 执行中收到各阶段 `progress`（阶段 15–18 使用 delivery 命名，无阶段 19），
      `stage_name` 与冻结命名一致。
- [ ] 任务真正完成（Action 返回 `success=true && completed_stage=20`）→ `result(state=completed)`
      且 `rehearsal=false`。
- [ ] 三类货品（food / daily / electronics）各完整走一遍，确认都能到达 stage 20。

### 4.4 异常与失败路径（fail-closed 验证）

- [ ] 任务中途失败（如抓取失败）：Action 返回非 20 阶段 → `result(state=failed)`，error_code 如实透传。
- [ ] 任务超时（`action.task_timeout`，默认 300 s）：返回 `failed`，不得返回 `completed`。
- [ ] 人工 preempt（新目标打断）：返回 `failed`，error_code=8（MISSION_ERROR_TASK_PREEMPTED）。
- [ ] **伪造成功防护**：若车端/Action 异常返回 `success=true` 但 `completed_stage=14`（或非 20），
      bridge 必须返回 `failed`——请务必实测这一条，这是规则红线。

### 4.5 去重 / 冲突 / 忙碌

- [ ] 同一 `request_id` + 相同内容重发：不重复执行，车端收到缓存补发的 ack/progress/result。
- [ ] 同一 `request_id` + 不同内容重发：收到 `error(error_code=8, reason=request_conflict)`。
- [ ] 上一请求未结束时发新请求：收到 `error(error_code=9, reason=busy)`。

### 4.6 断线 / 消息边界

- [ ] **无下推**：重连上升沿只收到一条 heartbeat，车端不重发请求则**收不到任何 result**
      （残留结果绝不主动推送）。
- [ ] 断线期间任务完成：重连后 result 经离线队列**恰好补发一次**；若车端重发
      pending_request，则从去重缓存应答，同一结果不会双发（`drop_pending` 兜底）。
- [ ] 车端断线重连后重发 pending_request → 收到缓存 ack/progress/result（恢复到断点）。
- [ ] 车端发送超过 64 KiB 的单行 → 被丢弃且不影响后续消息。
- [ ] 车端发送非法 JSON / 未知 message_type / 错误 schema_version → 收到对应 `error` 或消息被拒，
      节点不崩溃。
- [ ] 高频率消息（如 50 Hz progress）持续 5 分钟：无内存增长、无丢行、线程无泄漏。

### 4.7 端到端验收

- [ ] 按 2.1 正式入口整套启动（含 `start_bridge:=true`），实体车自动完成 任务2 播报 → 发请求 →
      仿真执行 → 返回 completed → 车端播报任务3 → 进入任务4 的全流程。
- [ ] 全程观察 `/simulation/bridge_status`，确认状态机（ready/busy/last_result 等）与预期一致。
- [ ] 联调过程保存：车端日志、仿真侧 `roslaunch` 日志、录屏，作为验收材料。

## 5. 遗留事项

- 跨机真实 TCP 通信（4.1）与端到端全流程（4.7）未做——本机无实体车可联调，属预期。
- Action server 真实反馈下的 stage 名称映射、Gazebo 暂停就绪失效的实际表现，本机按墙钟逻辑
  实现并通过单测 + mock 车端（2.5）演练，建议联调时现场复核。
- 最终录像需在原生 Ubuntu 20.04 仿真电脑上重新生成（画面含 Gazebo / RViz / 完整抓放过程 /
  real_time_factor ≤ 1 / 腾讯会议共享）；开发侧已提供 mock 车端、一键启动入口与测试步骤（2.5、4.x）。
- 桌面单测命令使用 `python3 -W error::ResourceWarning -m unittest discover -s test`（环境无 pytest）；
  README 中已同步。

## 6. 版本记录（v2.2，TEB_test_bridge_fix）

### 6.1 第一阶段：bridge 修改（本分支全部改动，未触碰其他包）

| 修改 | 内容 |
|---|---|
| laser_ready | heartbeat 新增 `"laser_ready"`：新鲜（墙钟 2 s 内）+ 结构有效的 /scan（非空、range 边界合法、无 NaN），**不代表检测到障碍**；保留 `sensors_ready`；`ready` 的 AND 门加入 laser（缺 scan 探针即 fail-closed） |
| 单调墙钟 | `TopicFreshness`/`LaserFreshness` 与 Action 总超时改用 `time.monotonic()`；TCP 重连退避本就为墙钟；`header.stamp`、TF 保持 `rospy.Time`。Gazebo 暂停 → `/scan` 停发 → 数秒后 `ready=false`；已开始的任务暂停中也会墙钟超时返回 `failed` |
| 任务等待纯墙钟（队友核查补充） | 弃用 `wait_for_result(rospy.Duration(1.0))`——其剩余时间由仿真时钟计算，暂停时永久阻塞、墙钟超时无法执行；改为 **done callback + `threading.Event.wait(1.0)`**，全部等待为墙钟 |
| Action server 探测墙钟化（队友核查补充） | `_check_action_server` 弃用 `wait_for_server(rospy.Duration(0.5))`——截止时间同样由仿真时钟计算，服务端未连上 + 暂停时就绪线程永久挂起；改为墙钟轮询 `is_server_connected()`（0.5 s 截止、50 ms 步长），就绪线程保持 fail-closed |
| gazebo_ready 需 /clock 活跃（队友核查补充） | `_check_gazebo` 在主题存在 + 服务注册之外，增加 **/clock 墙钟新鲜度探针**（`gazebo.clock_max_age` 默认 1 s）；暂停 → 主题仍在但停发 → 1 s 内 `gazebo_ready=false` |
| 删除无条件补发 | `_on_link_state` 重连上升沿仅补发心跳；结果只经"离线队列 flush 一次"或"车端重发请求 → 去重缓存应答"两条路径；`drop_pending` 移除队列中与缓存重放重复的副本，杜绝同一结果双发；无请求不推送残留结果 |
| ResourceWarning | 客户端 `_connect_once` 任何失败路径确定性关闭 socket；测试 handler 全部 try/finally 关闭连接；全套以 `-W error::ResourceWarning` 运行通过 |
| 配套 | 新增 `src/smart_factory_bridge/readiness.py`（ROS-free：FreshnessState / laser_scan_is_valid / build_heartbeat）、`test/test_readiness.py`（10 项，含墙钟等待回归防护）、tcp 新增 2 项测试、`tools/mock_vehicle_server.py`、package.xml 增加 `sensor_msgs` 依赖 |

### 6.2 第二阶段：路线合规使用清单（零修改，合规结论已确认）

`docs/路线合规使用清单.md`：固定坐标（delivery_goals.yaml 仓库口与准备位姿、pickup_staging_fitted_path.yaml、
fitted_waypoints）、拟合路线（fitted_path.py / route_executor.py / action_server.py / plot_fitted_navigation_path.py）、
动态中间 goal（delivery_entry_selector.py 扇形候选与滚动点、mission_server.py `_navigate_delivery_channel`）、
`/move_base/make_plan` 筛选（delivery_entry_selector.py 的 GetPlan 调用与候选加权）——全部带 file:line。

**合规结论（2026-08-18 确认）：规则允许设定中途导航点**——固定坐标与拟合路线的执行点、
动态中间 goal 均属中途导航点范畴，合规；`/move_base/make_plan` 仅调用官方全局规划器
筛选候选路线，无二次规划、无干预输出。四类用法**零代码修改、零调参**，维持现状。

### 6.3 第三阶段：放置验证方案（按队友最终决定，不做视觉二次验证）

- 正式删除"放置后视觉二次验证 / 仓库内物块识别模块"的开发计划。
- 完成链路（与 `mission_server.py` 现有实现一致，**mission 零修改**）：
  导航到正确仓库 → 到达允许的释放位姿（delivery_goals.yaml 固定口 + 容差）→ 机械臂释放
  → 释放动作正常结束、夹爪回到打开/空闲（`_pickup_pipeline.release` 成功）→ `TASK_COMPLETED`
  → bridge 转发 `completed`。任一步超时 / Action 失败 / 导航失败 / 释放失败（`PickupFailure`、
  `GoalUnavailable`、preempt）一律 `failed`，绝不进入 stage 20。
- 不用 Gazebo 真值替代验证：bridge 与 mission 均无 `/gazebo/model_states`、`get_model_state`
  或固定物块坐标作为完成判据（红线扫描已确认）。
- **稳定性测试计划**（后续主要通过仿真实测保证实际放置可靠，而非程序内验证）：
  1. 调整仓库内释放位姿、机械臂高度与朝向；
  2. 释放后留出短暂稳定时间再收回机械臂；
  3. 测试物块是否反弹、滑出或卡住（模拟释放指令成功但裁判判 0 分的场景）；
  4. 三类货品（food / daily / electronics）各连续多轮；
  5. 每轮重新运行官方 `spawn_cubes.py` 后复测；
  6. 人工通过腾讯会议中的 Gazebo 画面确认实际放置效果（录像要求见 5）。

## 7. 交付记录（2026-08-18，TEB_test_bridge_fix）

| 项 | 结果 |
|---|---|
| 分支 | `TEB_test_bridge_fix`（基线 `TEB_test@39d750c`，原分支未被覆盖） |
| 提交 | `1f582c5`（第一阶段 bridge 修改 + 测试 + mock 工具）；`7197a82`（阶段二合规清单）；`333cde9`/`0ad14f0`（本文档 v2/v2.1）；`dfc04f0`（队友核查修复：任务等待与 Action server 探测墙钟化、gazebo_ready 需 /clock 活跃、回归测试）；`（本文档 v2.2：合规结论与真值接口处置定案，待推送）` |
| 推送 | 已推送至 GitHub：`origin/TEB_test_bridge_fix @ 0ad14f0`；`dfc04f0` 与本文档 v2.2 **待推送**（推送由提交人完成） |
| 编译 | `catkin_make` 100% 成功（含新增 `sensor_msgs` 依赖） |
| 桌面单测 | 51 项全部通过，`-W error::ResourceWarning` 下 socket 零泄漏 |
| heartbeat 示例 | 见 §3.1 引用块（含 `laser_ready`、`sensors_ready`、`ready`、`busy`） |
| Gazebo 暂停测试 | 单测级暂停模拟已过（§3.1）；真机步骤见 §4.2，配合 mock 车端（§2.5）观察 |
| 断线重连测试 | TCP 层 6 项真实 socket 测试（§3.1）；mock 车端 `--crash-after` 演练步骤见 §2.5 |
| 三类货品完整录像 | **待原生 Ubuntu 仿真电脑生成**（要求见 §5/§6.3） |

### 7.1 剩余风险清单

1. **任务墙钟等待**：done callback + `threading.Event.wait()` 已在代码层修复
   `wait_for_result` 暂停卡死问题；真机联调时按 §4.2 用"暂停中任务超时返回 failed"
   实测确认。
2. **gazebo_ready 语义**：/clock 墙钟新鲜度探针已实现（默认 1 s）；暂停行为
   （`gazebo_ready=false` 在 1 s 内）真机联调现场复核（§4.2）。
3. **laser_ready"基本有效"定义**：当前为结构校验（非空、range 边界合法、无 NaN，
   全 inf 无回波仍有效）；若车端有更强要求（如最小光束数），联调时调整。
4. ~~阶段二合规结论待定~~ **已闭环（2026-08-18）**：规则允许设定中途导航点，
   固定坐标 / 拟合路线执行点 / 动态中间 goal 合规，`/move_base/make_plan` 仅筛选
   候选；四类用法零代码修改、零调参，明细见 `docs/路线合规使用清单.md`（§6.2）。
   联调期间若裁判现场提出异议，按结论修改后重测。
5. **Gazebo 真值接口处置已定（2026-08-18）**：`car3/scripts/grasp_attach.py` 为官方
   car3 包自带抓取随动机制，保持原样；`smart_factory_tests/scripts/monitor_navigation_safety.py`
   为测试监控脚本，**正式比赛前从仓库移除**（当前保留供联调排查，§1.4）。
6. **阶段三稳定性实测待做**：释放位姿与稳定时间调参、反弹/滑出测试、三类货品多轮
   复测、每轮重跑官方 `spawn_cubes.py`（计划见 §6.3）。
7. **车端契约联调**：`ready` 字段集（含 `laser_ready`）与重连行为（车端重发
   pending_request）需与 `simulation_link_node.py` 实际实现现场核对（§4 清单）。
