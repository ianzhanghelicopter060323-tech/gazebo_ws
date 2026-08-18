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
  0.5/1/2/4/5 s 指数退避重连。
- `ready` 由 Gazebo、RViz、Action server、定位（`/amcl_pose`）、雷达
  （`/scan`）新鲜度和 `busy` 共同决定；任何一项不满足即 `ready=false`
  （fail-closed，车端据此阻塞发车）。Gazebo 暂停时仿真时钟停走，
  传感器新鲜度自动失效。
- 同一 `request_id` + 相同内容重复到达：不重复执行，补发缓存
  ack/progress/result；同 ID 不同内容：`request_conflict`；执行期间新请求：
  `busy`。
- 只有 `success=true && completed_stage=20` 才发送 `completed`；阶段 14、
  aborted、preempted、超时一律如实发送 `failed`，绝不伪装成功。
- TCP 短暂断开后重连会补发一次缓存的最新结果；结果缓存上限
  `result_cache.max_entries`（默认 64）。
- 所有异常只记录不退出；`/simulation/bridge_status` 发布本机状态 JSON
  （调试用，车端不读取）。

## 桌面单测（不依赖 ROS Master）

```bash
cd src/smart_factory_bridge
python3 -m pytest test/ -v
```

覆盖：非法 JSON/超长/缺字段/坏 schema/未知类别拒绝；往返字段一致；
同 ID 去重与冲突指纹；阶段 20 严格判定；真实 socket 心跳往返、断连重连、
离线队列补发、超长行丢弃。
