# Gmapping 参数调整记录

## 变更信息

- 日期：2026-07-14
- 配置文件：`~/gazebo_ws/src/gazebo_map/launch/gmapping.launch`
- 原始备份：`~/gazebo_ws/src/gazebo_map/launch/gmapping.launch.backup-20260714`
- 目的：减小建图运动更新间隔，并降低地图高频发布造成的负载，以改善转向时的墙面跳变和地图畸变。

## 参数变化

| 参数 | 原始值 | 新值 | 说明 |
|---|---:|---:|---|
| `map_update_interval` | `0.01` | `1.0` | 将地图发布间隔调整为 1 秒，降低地图重建和发布负载 |
| `linearUpdate` | `0.5` | `0.1` | 小车每移动约 0.1 米触发一次扫描更新 |
| `angularUpdate` | `0.436` | `0.1` | 小车每旋转约 0.1 弧度（5.7°）触发一次扫描更新 |
| `temporalUpdate` | `-1.0` | `0.5` | 即使运动量较小，也允许每 0.5 秒触发一次扫描更新 |

## 生效方法

参数由 `slam_gmapping` 启动时读取，因此修改后需要结束并重新启动 gmapping：

```bash
roslaunch gazebo_map gmapping.launch
```

重新建图前建议删除或忽略此前已经畸变的地图显示；本次修改不会自动修复已经生成的地图。

## 恢复原始配置

如需回退，执行：

```bash
cp ~/gazebo_ws/src/gazebo_map/launch/gmapping.launch.backup-20260714 \
   ~/gazebo_ws/src/gazebo_map/launch/gmapping.launch
```

恢复后重新启动 gmapping。

---

## 第二轮：雷达覆盖范围与错误匹配抑制

### 变更信息

- 日期：2026-07-14
- 修改文件：`~/gazebo_ws/src/car3/urdf/car3.urdf`
- 修改文件：`~/gazebo_ws/src/gazebo_map/launch/gmapping.launch`
- 本轮 URDF 原始备份：`~/gazebo_ws/src/car3/urdf/car3.urdf.backup-20260714-before-slam-sensor-tuning`
- 本轮 gmapping 原始备份：`~/gazebo_ws/src/gazebo_map/launch/gmapping.launch.backup-20260714-before-slam-sensor-tuning`
- 说明：本轮 gmapping 备份包含第一轮已经调整过的地图更新参数，可用于只回退第二轮修改。

### 参数变化

| 文件 | 参数 | 本轮修改前 | 本轮修改后 | 说明 |
|---|---|---:|---:|---|
| `car3.urdf` | 激光雷达 `update_rate` | `15 Hz` | `20 Hz` | 提高扫描刷新率 |
| `car3.urdf` | `min_angle` | `-1.57 rad` | `-3.14159 rad` | 将左侧扫描边界扩展到约 -180° |
| `car3.urdf` | `max_angle` | `1.57 rad` | `3.14159 rad` | 将右侧扫描边界扩展到约 180°，形成 360°视场 |
| `gmapping.launch` | `maxRange` | `5.0 m` | `7.0 m` | 保留当前场地中较远墙面的有效扫描 |
| `gmapping.launch` | `maxUrange` | `4.5 m` | `6.5 m` | 扩大实际用于建图的扫描距离 |
| `gmapping.launch` | `minimumScore` | 未设置（默认 `0`） | `50` | 拒绝得分过低的扫描匹配，抑制姿态突然跳变 |

雷达仍使用 `720` 个采样点；视场扩展到 360°后，角分辨率约为 0.5°。

### 生效方法

URDF 由 Gazebo 生成模型时读取，因此需要结束并重新启动 Gazebo；随后重新启动 gmapping：

```bash
roslaunch car3 gazebo.launch
```

```bash
roslaunch gazebo_map gmapping.launch
```

已经产生畸变的地图不会被参数修改自动修复，应在重启 gmapping 后重新建图。

### 只回退第二轮修改

```bash
cp ~/gazebo_ws/src/car3/urdf/car3.urdf.backup-20260714-before-slam-sensor-tuning \
   ~/gazebo_ws/src/car3/urdf/car3.urdf

cp ~/gazebo_ws/src/gazebo_map/launch/gmapping.launch.backup-20260714-before-slam-sensor-tuning \
   ~/gazebo_ws/src/gazebo_map/launch/gmapping.launch
```

回退后同样需要重新启动 Gazebo 和 gmapping。

---

## 第三轮：雷达视场恢复为180°

- 日期：2026-07-14
- 修改文件：`~/gazebo_ws/src/car3/urdf/car3.urdf`
- 360°测试版备份：`~/gazebo_ws/src/car3/urdf/car3.urdf.backup-20260714-360deg-test`
- 原因：360°测试期间地图变化剧烈；当前雷达安装在车体前方，后向扫描存在受到车体及机械臂碰撞几何干扰的风险。

| 参数 | 修改前 | 修改后 |
|---|---:|---:|
| `min_angle` | `-3.14159 rad` | `-1.57 rad` |
| `max_angle` | `3.14159 rad` | `1.57 rad` |

本次只恢复扫描视场，以下第二轮参数继续保留：

- 雷达 `update_rate=20 Hz`
- gmapping `maxRange=7.0 m`
- gmapping `maxUrange=6.5 m`
- gmapping `minimumScore=50`

URDF 修改后需要完全重启 Gazebo，已经生成的机器人模型不会自动应用新视场。

---

## 第四轮：使用 Gazebo 真值里程计锁定位姿

- 日期：2026-07-23
- 修改文件：`~/gazebo_ws/src/gazebo_map/launch/gmapping.launch`
- 修改前备份：`~/gazebo_ws/src/gazebo_map/launch/gmapping.launch.backup-20260723-before-odom-locked-mapping`
- 原因：对 `laser_collapse.bag` 的逐帧分析表明，`odom -> base_footprint` 连续平滑，突变来自 gmapping 发布的 `map -> odom`。最大一次在仿真时间 `778.572 s` 的 50 ms 内平移 `1.088 m`、旋转 `0.701 rad`。异常前后 720 束激光均有效，且底盘几乎静止，说明是重复场景中的高分错误扫描匹配，而不是坏扫描或里程计突变。

当前底盘由 `gazebo_ros_planar_move` 驱动，其 `/odom` 直接使用 Gazebo 世界位姿，没有轮式里程计的累计漂移。因此本轮让 gmapping 只负责把激光写入栅格地图，不再允许扫描匹配修正精确的仿真位姿。

| 参数 | 修改前 | 修改后 | 说明 |
|---|---:|---:|---|
| `lstep` | `0.02 m` | `0.0 m` | 禁止扫描优化器平移候选位姿 |
| `astep` | `0.02 rad` | `0.0 rad` | 禁止扫描优化器旋转候选位姿 |
| `iterations` | `5` | `1` | 零步长下只保留一次原位评分 |
| `minimumScore` | `80` | `0.0` | 接受原位评分，避免把 odom-only 模式记录为匹配失败 |
| `srr` | `0.001` | `0.0` | 关闭平移-平移运动噪声 |
| `srt` | `0.002` | `0.0` | 关闭旋转-平移运动噪声 |
| `str` | `0.001` | `0.0` | 关闭平移-旋转运动噪声 |
| `stt` | `0.002` | `0.0` | 关闭旋转-旋转运动噪声 |
| `linearUpdate` | `0.1 m` | `0.05 m` | 每移动约 5 cm 插入一次扫描 |
| `angularUpdate` | `0.1 rad` | `0.05 rad` | 每旋转约 2.9 度插入一次扫描 |
| `temporalUpdate` | `0.5 s` | `-1.0` | 禁止静止时仅因定时器触发扫描匹配 |
| `resampleThreshold` | `0.7` | `0.5` | 单粒子模式下保留合法常规值 |
| `particles` | `80` | `1` | 精确里程计下取消粒子间轨迹竞争和最佳粒子切换 |

这组参数专用于当前的 Gazebo 真值里程计。若以后换成真实轮式里程计或带累计漂移的仿真里程计，应恢复备份并重新调试扫描匹配，而不能继续使用本模式。

参数需要重启 `slam_gmapping` 后才会生效；不需要重启 Gazebo。重启后应重新开始建图，因为旧进程中的地图已经包含此前的错误轨迹。

验证：在隔离 ROS master 中完整回放同一份 796 秒异常 bag，并过滤 bag 自带的旧 `map -> odom` 后，新配置共发布 15,911 次 `map -> odom`。最大单步平移为 `2.10e-14 m`，最大单步旋转为 `1.18e-14 rad`，均为浮点舍入误差；原先在 `409–412 s` 和 `778.572 s` 的突变均未复现。

---

## 第五轮：恢复赛事仓库初始定位参数

- 日期：2026-07-24
- 基线：本机 Git 仓库 `/home/ianichinose/iflytek-smart-car` 的 `origin/main`
- 基线提交：`4fbf065`（`Add Gazebo simulation guide and official workspace`）
- 官方目录：`Gazebo/gazebo_ws/src`
- 恢复原因：第四轮配置会完全信任 Gazebo 世界位姿产生的 `/odom`，并关闭 GMapping 的扫描位姿搜索和运动不确定性，不符合当前赛事对自主定位的要求。

已恢复并与 Git 基线交叉核对：

- `src/gazebo_map/launch/gmapping.launch`
- `src/car3/urdf/car3.urdf` 中的激光雷达 `update_rate`
- `src/gazebo_nav/launch/config/amcl/amcl_omni.launch`

恢复后的 GMapping 关键参数为：

| 参数 | 恢复值 |
|---|---:|
| `map_update_interval` | `0.01` |
| `maxRange` | `5.0` |
| `maxUrange` | `4.5` |
| `lstep` | `0.05` |
| `astep` | `0.05` |
| `iterations` | `5` |
| `srr` | `0.01` |
| `srt` | `0.02` |
| `str` | `0.01` |
| `stt` | `0.02` |
| `linearUpdate` | `0.5` |
| `angularUpdate` | `0.436` |
| `temporalUpdate` | `-1.0` |
| `resampleThreshold` | `0.5` |
| `particles` | `80` |

同时恢复激光雷达 `update_rate=15 Hz`，并恢复 AMCL 官方运动模型、里程计噪声、激光模型和粒子参数。第四轮仅作为历史记录保留，不再代表当前生效配置。
