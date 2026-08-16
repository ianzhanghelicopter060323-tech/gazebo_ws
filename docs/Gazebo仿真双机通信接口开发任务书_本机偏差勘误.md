# Gazebo 仿真双机通信接口开发任务书 · 本机偏差勘误

> 1 2节无关紧要、已删除
>
> 用途：记录 `docs/Gazebo仿真双机通信接口开发任务书.md`（下称「任务书」）与本仿真工作空间的表述差异。
> **凡任务书描述与本工作空间不一致处，一律以本机为准。**
>
> 基准快照：2026-08-16（`src/` 下 `smart_factory_*` 各包当前代码）。
> 说明：本工作空间是**仿真电脑侧**；实体车侧代码不在本仓库，见第 2 节。

## 3. 状态机差异（任务书 §7 对照）

| 项 | 任务书 §7 | 本机（`states.py` / `ExecuteTask.action`） | 结论 |
| --- | --- | --- | --- |
| 阶段 19 | `19 VERIFY_RELEASE` | **不存在**；`18 RELEASE_OBJECT` 后直接到 `20 TASK_COMPLETED`，放置验证折叠在 release 内 | 以本机为准：阶段序列为 0–18、20、250，**无 19** |
| 阶段 15–18 命名 | `GET_WAREHOUSE_GOAL`(15) / `NAVIGATE_TO_WAREHOUSE`(16) / `ARRIVED_WAREHOUSE`(17) / `RELEASE_OBJECT`(18) | `GET_DELIVERY_GOAL`(15) / `NAVIGATE_TO_DELIVERY`(16) / `ARRIVED_DELIVERY`(17) / `RELEASE_OBJECT`(18) | 数值一致、名字不同（warehouse→delivery）；bridge 的 progress `stage_name` 以本机名为准 |
| 阶段 20 | `20 TASK_COMPLETED` | `20 TASK_COMPLETED` | 一致，成功终点 |
| 失败终点 | 未定义 | `250 TASK_FAILED` | 本机新增，bridge 的 failed result 由此给出 |

## 4. 任务书 §4.2 / §13「现状」描述已过期

任务书写作时基于更旧的代码快照，以下均以本机为准：

| 任务书表述 | 本机实际 |
| --- | --- |
| §4.2 `ExecuteTask.action`「反馈阶段只定义到 OBJECT_GRASPED=14」 | feedback 常量已完整定义 15/16/17/18/20/250 |
| §4.2 `mission_server.py`「当前成功终点是 OBJECT_GRASPED，不是完整任务」 | `_continue_after_arrival` 已实现完整链路：抓取 → 运输（滚动通道避障）→ 放置 → `TASK_COMPLETED` |
| §4.2 `mission.yaml`「当前设置 pipeline_stop_after: OBJECT_GRASPED」 | 已是 `pipeline_stop_after: TASK_COMPLETED` |
| §4.2 机械臂「抓取已有，运输与放置待补」 | 运输（`GET_DELIVERY_GOAL`→`NAVIGATE_TO_DELIVERY`→`ARRIVED_DELIVERY`）与放置（`RELEASE_OBJECT`→`TASK_COMPLETED`）已实现 |
| §13「仿真状态机从 OBJECT_GRASPED 补到 TASK_COMPLETED」 | 该补齐**已完成**，不属本接口任务新增范围 |
| §4.2 `smart_factory_bridge/`「只含空的 package.xml、CMakeLists.txt 和 README」 | 仍为空壳（仅占位文件），**通信实现尚未交付**，是当前唯一主要缺口 |

## 5. 失败后车端行为（任务书 §6.6 / §8 / §11 作废，以本决策为准）

任务书原要求 fail-closed：

- §6.6「车端收到 failed 后停车并结束本轮自动链路，不播报『仿真任务已完成』，也不进入任务 4」
- §8「任一校验失败都保持停车、发布明确错误状态，绝不能降级为成功」
- §11.4「失败场景均 fail-closed：停车、不误播、不继续」

**本决策（2026-08-16 用户确认，替代上述条目）：fail-open。**

> 实体车对任务 3，无论收到 `result.completed`、`result.failed`、等待超时还是 TCP 掉线（仿真卡死/退出），**一律当作「仿真任务已完成」**：照常播报成功语音「仿真任务已完成，已将［仿真货品名称］放入［仓库类别］」，并继续进入后续任务（任务 4 交通灯），以保证全程完整性、不被仿真卡死拖死整场。

落地约定：

1. 车端（车侧代码）实现**有界等待**：正常 `completed` → 播报成功、进任务 4；`failed` / 超时 / 掉线 → 同样播报成功、进任务 4。
2. 仿真侧 bridge **永远如实上报**：`completed` 只在 `success=true && completed_stage=20` 时发送；失败/进度照实发送；仿真卡死时连接自然断开。车端靠超时兜底，不依赖仿真侧伪装成功。
3. 任务书 §11.1（`OBJECT_GRASPED=14` 不能让实体车播报或继续）、§11.3 第 4/5 条（确认实体车拒绝继续 / 保持停车不误播）、§11.4（fail-closed）按新语义失效，验收按本勘误执行。
4. 已确认接受的风险：失败时仍播报成功语音，播报内容可能与共享仿真画面不符，存在被裁判判为造假的合规风险。

## 6. 消息格式补充定义（任务书未冻结，以本机为准）

任务书 §6 冻结了 heartbeat / request / ack / result / error 的字段，但 **progress 字段未定义**。本机 bridge 按以下格式实现并视为冻结：

```json
{
  "schema_version": 1,
  "message_type": "progress",
  "session_id": "<sim-uuid>",
  "request_session_id": "<car-uuid>",
  "request_id": "<order_id>:simulation:<uuid>",
  "order_id": "<本轮订单编号>",
  "stage": 16,
  "stage_name": "NAVIGATE_TO_DELIVERY",
  "retry_count": 0,
  "detail": "<feedback.detail>",
  "timestamp": 0.0
}
```

其余消息格式与任务书 §6.2–§6.6 一致，不在此重复。
