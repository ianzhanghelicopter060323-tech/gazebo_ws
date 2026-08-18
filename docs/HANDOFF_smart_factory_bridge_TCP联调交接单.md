# smart_factory_bridge 双机 TCP 通信接入与联调交接单

> 编写日期：2026-08-18
> 接收人：Gazebo 仿真任务开发队友 / 实体车联调队友
> 目标环境：原生 Ubuntu 20.04 + ROS Noetic + Gazebo Classic + RViz
> 当前分支：`TEB_test`
> 协议基线：`docs/Gazebo仿真双机通信接口开发任务书.md` 第 6 节
> 　　＋ `docs/Gazebo仿真双机通信接口开发任务书_本机偏差勘误.md`（阶段 15–18 delivery 命名、无阶段 19、progress 格式冻结）
> 当前结论：**仿真侧 bridge 包已实现完毕，39 项桌面单测全部通过；尚未与实体车做真实局域网联调。** 下文给出使用方法和联调测试清单。

## 1. 当前进度

### 1.1 已交付内容

`src/smart_factory_bridge/` 包（仿真电脑侧双机通信桥）：

```text
scripts/vehicle_bridge_node.py         # ROS 节点：NDJSON 请求 ↔ /sim_task/execute Action ↔ 就绪检查
src/smart_factory_bridge/protocol.py   # NDJSON 编解码、严格校验、请求指纹、ack/progress/result/error 构造
src/smart_factory_bridge/tcp_client.py # 心跳、退化/断连检测、指数退避重连、离线发送队列
src/smart_factory_bridge/action_adapter.py  # Action feedback/result → progress/result 消息映射
config/bridge.yaml                     # 端口、超时、传感器新鲜度、RViz 检查、结果缓存等配置
launch/bridge.launch                   # 独立启动入口
test/                                  # 桌面单测（39 项，不依赖 ROS Master）
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
| 心跳与就绪 | 每 1 s 发 heartbeat；`ready` 由 Gazebo、RViz、Action server、定位、雷达新鲜度和 `busy` 共同决定，任一不满足即 `ready=false`（fail-closed，车端据此阻塞发车）；Gazebo 暂停 → 仿真时钟停走 → 传感器新鲜度自动失效 |
| 链路退化/重连 | 3 s 无数据 → degraded；10 s → 断开并按 0.5/1/2/4/5 s 指数退避重连 |
| 去重 | 同一 `request_id` + 相同内容重复到达：不重复执行，补发缓存 ack/progress/result |
| 冲突 | 同一 `request_id` 不同内容：`request_conflict`（ERR_REQUEST_CONFLICT） |
| 忙碌 | 执行期间新请求：`busy`（ERR_BUSY） |
| 成功判定 | **只有 `success=true && completed_stage=20` 才发送 `completed`**；阶段 14、aborted、preempted、任务超时一律如实发送 `failed`，绝不伪装成功（fail-closed 是桥的职责，fail-open 是车端职责） |
| 断线补发 | TCP 断开重连后补发一次缓存的最新 result；结果缓存上限 `result_cache.max_entries`（默认 64） |
| 健壮性 | 所有异常只记录不退出；`/simulation/bridge_status` 发布本机状态 JSON（调试用，车端不读取） |

### 1.4 约定与红线（代码层面已保证）

- 无 `/gazebo/model_states`、无第二个全局规划器（使用官方全局规划器）。
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
| `degraded_after` / `disconnect_after` | 3.0 / 10.0 s | 退化 / 强制断开阈值 |
| `reconnect_backoff` | [0.5, 1, 2, 4, 5] s | 指数退避序列（封顶） |
| `max_line_bytes` | 65536 | 单行消息上限 |
| `sensor_topics` | /scan、/amcl_pose | 新鲜度探测主题（仿真时钟，Gazebo 暂停即失效） |
| `gazebo` | /clock + /gazebo/get_physics_properties | 就绪判定 |
| `rviz` | mode: process（pgrep 进程探针） | RViz 就绪判定 |
| `action` | /sim_task/execute，wait 15 s，任务超时 300 s | Action 配置 |
| `result_cache.max_entries` | 64 | 结果缓存上限 |

### 2.4 桌面单测（不依赖 ROS Master）

```bash
cd /home/ianzhang/gazebo_ws/src/smart_factory_bridge
python3 -m unittest discover -s test
# 期望输出：Ran 39 tests ... OK
```

## 3. 本机（仿真侧）已进行的测试

### 3.1 单测覆盖（39 项，全部通过，连续 4 次运行稳定，单次约 1.8 s）

| 模块 | 数量 | 覆盖点 |
|---|---|---|
| `test_protocol.py` | 21 | 编解码往返；超长/非法 JSON/非对象/坏 schema/未知类型/缺 session 拒绝；request 校验（缺字段/类型错误/未知类别/类别与 target_class 不一致/未知任务类型）；请求指纹（同内容同指纹——timestamp/issued_at 不计入；不同内容不同指纹）；ack/progress/result/error 构造均回显 request_session_id/request_id/order_id/target_class；result 仅在 `success && stage==20` 时为 completed；无 request 的 error 构造 |
| `test_action_adapter.py` | 12 | ack；feedback→progress 映射（阶段 16→NAVIGATE_TO_DELIVERY，15–18 delivery 命名）；SUCCEEDED+stage 20→completed；**SUCCEEDED+stage 14 必须为 failed**；success=false 即使 stage 20 也为 failed；aborted 透传 error_code、无 error_code 用 255；preempted→8；lost→failed；身份字段恒回显；rehearsal 恒为 false |
| `test_tcp_client.py` | 4 | 真实 socket（127.0.0.1 监听端口）心跳往返与 ack 接收；服务端主动断开→客户端上报 disconnected；**断线重连后离线队列中的 result 补发成功**；超长行丢弃且下一行正常处理 |

### 3.2 静态与红线检查（已执行，全部通过）

- `py_compile` 全部 Python 文件（src、scripts、test）无语法错误。
- `bridge.launch`、`full_competition.launch` XML 解析通过。
- 红线扫描：无 `/gazebo/model_states`；生产代码无硬编码 IP（仅测试代码使用 127.0.0.1 环回）；无阶段 19；`completed` 仅在 `success && stage==20` 路径产生。

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

- [ ] 正常全套启动（Gazebo + RViz + 任务节点 + 定位 + 雷达）：heartbeat `ready=true`。
- [ ] Gazebo 暂停（`/clock` 停走）：数秒内 `ready` 变为 `false`，车端不得发车。
- [ ] 关掉 RViz（进程探针）：`ready=false`。
- [ ] 关掉 Action server 或 /scan、/amcl_pose 数据：`ready=false`。
- [ ] 任务执行期间 heartbeat `busy=true`。

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

- [ ] 任务执行中断开 TCP（拔网线）→ 重连成功后，车端收到**一次**补发的缓存 result
      （缓存上限 64 条，先入先出）。
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
- Action server 真实反馈下的 stage 名称映射、Gazebo 暂停就绪失效的实际表现，本机仅按仿真时钟
  逻辑实现并通过单测，建议联调时现场复核。
- 桌面单测命令使用 `python3 -m unittest`（环境无 pytest）；README 中已同步。
