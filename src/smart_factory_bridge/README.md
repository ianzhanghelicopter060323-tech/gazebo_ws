# smart_factory_bridge

仿真电脑侧双机通信桥。本包是 **TCP 客户端**，主动连接实体车的 TCP 服务端
（默认 `0.0.0.0:24580`），把实体车的 NDJSON 请求转成 `/sim_task/execute`
Action 目标，并把 Action 的 feedback/result 如实回传给实体车。

协议冻结版见 `docs/Gazebo仿真双机通信接口开发任务书.md` 第 6 节，以及
`docs/Gazebo仿真双机通信接口开发任务书_本机偏差勘误.md`（阶段 15–18 采用
delivery 命名、无阶段 19、progress 格式冻结）。

## 布局

```text
scripts/vehicle_bridge_node.py        # ROS 节点：协议 ↔ Action ↔ 就绪检查
src/smart_factory_bridge/protocol.py  # NDJSON 编解码、校验、指纹、消息构造
src/smart_factory_bridge/tcp_client.py# 心跳、退化/断连检测、指数退避重连、发送队列
src/smart_factory_bridge/action_adapter.py  # Action 事件 → ack/progress/result
launch/bridge.launch                  # 独立启动入口
config/bridge.yaml                    # 端口、超时、传感器新鲜度、RViz 检查等
test/                                 # 桌面单测（无需 ROS Master）
```

## 启动

正式入口是 `smart_factory_bringup/launch/full_competition.launch`：

```bash
roslaunch smart_factory_bringup full_competition.launch \
  start_bridge:=true \
  vehicle_host:=<实体车局域网IP> \
  vehicle_port:=24580
```

`vehicle_host` 也可以改用环境变量 `SIMULATION_VEHICLE_HOST`。IP 不允许写死在
源码或配置文件里。bridge 也可单独启动：

```bash
roslaunch smart_factory_bridge bridge.launch \
  vehicle_host:=<实体车局域网IP> vehicle_port:=24580
```

## 行为要点

- heartbeat 每 1 s 发送；3 s 收不到任何数据 → degraded；10 s → 断开并按
  0.5/1/2/4/5 s 指数退避重连（全部为单调墙钟计时）。
- `ready` 由 Gazebo、RViz、Action server、定位（`/amcl_pose`）、
  `laser_ready`（/scan 新鲜且基本有效）、传感器新鲜度和 `busy` 共同决定；
  任何一项不满足即 `ready=false`（fail-closed，车端据此阻塞发车）。
  Gazebo 暂停 → `/clock`、`/scan` 停止发布 → 墙钟新鲜度数秒内失效 →
  `ready=false`。已开始的任务在暂停中也会墙钟超时（默认 300 s）返回 `failed`。
- `laser_ready` 表示"最近收到新鲜且基本有效的 /scan"（非空、range 边界合法、
  无 NaN），**不是**"检测到障碍物"；与 `sensors_ready` 同时保留（车端兼容）。
- 同一 `request_id` + 相同内容重复到达：不重复执行，补发缓存
  ack/progress/result；同 ID 不同内容：`request_conflict`；执行期间新请求：
  `busy`。
- 只有 `success=true && completed_stage=20` 才发送 `completed`；阶段 14、
  aborted、preempted、超时一律如实发送 `failed`，绝不伪装成功。
- 重连上升沿**不主动推送残留结果**：断线期间产生的 result 经离线队列恰好
  补发一次；车端重发 pending_request 时由去重缓存应答；`drop_pending` 保证
  同一结果不会经"离线队列"与"缓存重放"发送两遍。结果缓存上限
  `result_cache.max_entries`（默认 64）。
- 新鲜度与任务超时全部使用 `time.monotonic()`；`header.stamp`、TF 查询等
  仍用 `rospy.Time`。
- 所有异常只记录不退出；socket 确定性关闭（ResourceWarning 零泄漏）；
  `/simulation/bridge_status` 发布本机状态 JSON（调试用，车端不读取）。

本地无实体车联调可运行 `tools/mock_vehicle_server.py` 扮演车端 TCP 服务端
（纯标准库）：`python3 tools/mock_vehicle_server.py --send-request --timeout 60`。

## 桌面单测（不依赖 ROS Master）

```bash
cd src/smart_factory_bridge
python3 -W error::ResourceWarning -m unittest discover -s test
```

覆盖：非法 JSON/超长/缺字段/坏 schema/未知类别拒绝；往返字段一致；
同 ID 去重与冲突指纹；阶段 20 严格判定；真实 socket 心跳往返、断连重连、
离线队列补发、重连不推残留结果、`drop_pending` 防双发、超长行丢弃；
墙钟新鲜度与 Gazebo 暂停模拟；`laser_ready` 结构有效性；heartbeat 字段
（`laser_ready`/`sensors_ready`/AND 门）。
