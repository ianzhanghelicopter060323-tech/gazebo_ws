# Costmap 膨胀半径调整记录

> 当前配置摘要（2026-07-28 第四轮，完整重启复测通过）：全局和局部 `inflation_radius` 均为 `0.15 m`，公共 `cost_scaling_factor=3.5`，局部滚动窗口为 `4.0 m × 4.0 m`；DWA 的 `acc_lim_theta=2.0 rad/s²`、`max_vel_theta=0.6 rad/s`、`min_vel_theta=0.1 rad/s`、`sim_time=1.0 s`、`xy_goal_tolerance=0.10 m`、`yaw_goal_tolerance=3.2 rad`。第 1～8 节保留此前调整历史，当前测试结论以第 9 节为准。

## 1. 变更信息

- 修改日期：2026-07-28
- 修改文件：`~/gazebo_ws/src/gazebo_nav/launch/gazebo_nav.launch`
- 修改文件：`~/gazebo_ws/src/gazebo_nav/launch/config/move_base/costmap_common_params.yaml`
- 修改后 `gazebo_nav.launch` SHA-256：`8562272cf5ba955089321a50babf581bc59ace8f6128eabffe759f7372d193c4`
- 修改后 `costmap_common_params.yaml` SHA-256：`2c7b4afe1ebf93f77f4807fa7605f8fa41ed0d62eb02f92065bc9362b629fb6e`
- 调整目的：允许全局代价地图和局部代价地图使用不同的膨胀半径，使全局路径与障碍物保持更大距离，同时保留局部规划在狭窄区域的机动空间。

## 2. 参数变化

| 项目 | 修改前 | 修改后 |
|---|---:|---:|
| 公共配置 `inflation_layer/inflation_radius` | `0.20 m` | 移除，由启动文件分别设置 |
| 全局最终膨胀半径 | `0.20 m` | `0.30 m` |
| 局部最终膨胀半径 | `0.20 m` | `0.20 m` |
| 公共 `cost_scaling_factor` | `3.5` | `3.5`，保持不变 |

本轮采用保守拆分：局部地图继续使用已经在用的 `0.20 m`，避免立即降低狭窄区域通过能力；只将全局地图增加到 `0.30 m`，让全局规划优先远离障碍物。

## 3. 参数加载与覆盖关系

`costmap_common_params.yaml` 仍由 `gazebo_nav.launch` 分别加载到 `global_costmap` 和 `local_costmap` 顶层命名空间，但公共文件不再包含 `inflation_radius`。

启动文件在所有 costmap YAML 加载完成后写入以下两个独立参数：

- `/move_base/global_costmap/inflation_layer/inflation_radius`
- `/move_base/local_costmap/inflation_layer/inflation_radius`

对应的启动参数及默认值为：

```xml
<arg name="global_inflation_radius" default="0.30"/>
<arg name="local_inflation_radius" default="0.20"/>
```

这样不会再由公共配置把两个半径绑定为同一个值。

## 4. 启动和临时调参

使用默认值启动：

```bash
roslaunch gazebo_nav gazebo_nav.launch
```

无需编辑文件即可在启动时临时覆盖，例如：

```bash
roslaunch gazebo_nav gazebo_nav.launch \
  global_inflation_radius:=0.35 \
  local_inflation_radius:=0.22
```

修改启动参数后需要重新启动 `move_base`。如果使用动态参数调整工具临时修改，重新启动后仍会恢复为启动文件参数。

## 5. 静态验证结果

2026-07-28 已完成：

1. `xmllint --noout` 检查通过，`gazebo_nav.launch` 是合法 XML。
2. `roslaunch --files gazebo_nav gazebo_nav.launch start_gazebo:=false start_rviz:=false` 解析通过。
3. `roslaunch --dump-params` 确认最终参数为：

```text
/move_base/global_costmap/inflation_layer/inflation_radius: 0.3
/move_base/local_costmap/inflation_layer/inflation_radius: 0.2
```

运行时仍需重点观察：

- 全局路径是否比修改前更少贴近墙体和静态障碍物；
- 局部规划器跟踪全局路径时是否出现振荡或频繁绕行；
- 狭窄通道能否继续通过；
- 机器人 footprint、定位误差和控制误差下是否仍有足够碰撞余量。

## 6. 回退方法

若要临时恢复修改前“两张地图均为 `0.20 m`”的行为，可直接启动：

```bash
roslaunch gazebo_nav gazebo_nav.launch \
  global_inflation_radius:=0.20 \
  local_inflation_radius:=0.20
```

若要永久恢复相同行为，只需将 `gazebo_nav.launch` 中 `global_inflation_radius` 的默认值从 `0.30` 改回 `0.20`；两个参数保持独立不会影响该行为。


---

## 7. 第二轮：修复 DWA `Off Map` 连续警告

### 7.1 变更信息

- 修改日期：2026-07-28
- 修改文件：`~/gazebo_ws/src/gazebo_nav/launch/config/move_base/local_costmap_params.yaml`
- 修改后 SHA-256：`2d355cf6c74dde92a002b95b957c8e776e0ef4ce5b7d8e395fd45dafa63b19dc`
- 修改范围：仅修改 ROS Navigation 原生局部代价地图参数并更新本记录。

### 7.2 根因证据

此前运行日志中的警告来自 `base_local_planner/map_grid_cost_function.cpp` 的 `MapGridCostFunction::scoreTrajectory`，典型输出为：

```text
Off Map 1.045975, 0.088854
Off Map 1.424769, 0.003268
```

这表示 DWA 候选轨迹的评分点越过局部代价地图，并不表示导航目标超出静态全局地图。修改前局部滚动窗口为 `2.0 m × 2.0 m`，机器人到窗口边缘只有约 `1.0 m`；而当前 DWA 的最远预测位移和前向评分偏移约为：

```text
max_vel_trans × sim_time + forward_point_distance
= 1.0 m/s × 1.5 s + 0.3 m
= 1.8 m
```

因此部分候选轨迹必然超出原窗口。膨胀参数拆分改变全局路径的代价分布后可能让该潜在尺寸矛盾更容易出现，但日志中的直接越界发生在局部 DWA 轨迹评分阶段，不是参数命名空间加载错误。

### 7.3 参数变化

| 参数 | 修改前 | 修改后 |
|---|---:|---:|
| `local_costmap/width` | `2.0 m` | `4.0 m` |
| `local_costmap/height` | `2.0 m` | `4.0 m` |
| `local_costmap/resolution` | `0.04 m` | `0.04 m`，保持不变 |
| 局部栅格数量 | `50 × 50` | `100 × 100` |

`4.0 m × 4.0 m` 窗口提供约 `2.0 m` 的半边长，可以容纳约 `1.8 m` 的最远评分点，同时栅格总数仅为 10,000，计算开销仍然较小。

以下用户后续调整原样保留，本轮没有覆盖：

- 全局 `inflation_radius=0.10 m`；
- 局部 `inflation_radius=0.10 m`；
- 公共 `cost_scaling_factor=4.0`；
- `obstacle_range=4.0 m`；
- `raytrace_range=4.0 m`；
- 全部 DWA 速度、采样和评分参数。

### 7.4 赛事规则边界

本轮修改符合 `docs/simu_requirement.md` 中“仅可通过原生参数配置文件调试内置参数”的范围：

- 没有修改官方全局规划器或 DWA 规划器源码；
- 没有新增或封装规划节点；
- 没有读取 `/gazebo/model_states` 或其他 Gazebo 真值接口；
- 没有硬编码赛道坐标、障碍物坐标或固定轨迹；
- 没有修改物块生成脚本及其他赛事禁止文件。

### 7.5 静态验证结果

`roslaunch --files` 解析通过，`roslaunch --dump-params` 的最终展开结果为：

```text
/move_base/global_costmap/inflation_layer/inflation_radius: 0.1
/move_base/local_costmap/height: 4.0
/move_base/local_costmap/inflation_layer/inflation_radius: 0.1
/move_base/local_costmap/resolution: 0.04
/move_base/local_costmap/width: 4.0
```

参数文件不会被已经运行的 `move_base` 自动重载。应在正式运行前完整重启导航，不在比赛运行中途动态修改参数：

```bash
roslaunch gazebo_nav gazebo_nav.launch
```

重启后可确认：

```bash
rosparam get /move_base/local_costmap/width
rosparam get /move_base/local_costmap/height
```

两项均应输出 `4.0`。运行时验证重点是下发导航目标后不再出现 `MapGridCostFunction::scoreTrajectory` 的连续 `Off Map` 警告。


---

## 8. 第三轮：膨胀关系与 DWA 参数调整（待运行验证）

### 8.1 变更信息

- 修改日期：2026-07-28
- 参数修改文件：
  - `~/gazebo_ws/src/gazebo_nav/launch/gazebo_nav.launch`
  - `~/gazebo_ws/src/gazebo_nav/launch/config/move_base/dwa_local_planner_params.yaml`
  - `~/gazebo_ws/src/gazebo_nav/launch/config/move_base/costmap_common_params.yaml`
- 本轮仅记录和检查 ROS Navigation 官方规划器的原生参数；没有修改规划器源码、增加干预节点或改写规划输出。
- 本轮检查只更新本文档，没有改动上述导航配置文件。

检查时各配置文件 SHA-256：

```text
85a73c2812082a417ffc9727697ab385a90d88d0e98f4aa5708d74fe43c13c32  gazebo_nav.launch
f5252cebcebca4d4ac578f42dcb42f87891e9a002be94b7f870332674bcc8993  dwa_local_planner_params.yaml
24b8530d6d0ad4c8ab2cb70a7dca03f8f0173080c358f1b4ad013580fe2a943f  costmap_common_params.yaml
2d355cf6c74dde92a002b95b957c8e776e0ef4ce5b7d8e395fd45dafa63b19dc  local_costmap_params.yaml
```

### 8.2 本轮参数变化

以下变化以本轮调整前的最近一次检查值为基准：

| 参数 | 调整前 | 当前值 | 目的 |
|---|---:|---:|---|
| 全局 `inflation_radius` | `0.15 m` | `0.20 m` | 让全局路径提前避开静态障碍物 |
| 局部 `inflation_radius` | `0.20 m` | `0.15 m` | 保留狭窄通道内的局部机动空间 |
| DWA `min_vel_theta` | `1.0 rad/s` | `0.4 rad/s` | 降低末端旋转过猛和越过目标角度的风险 |
| DWA `occdist_scale` | `0.01` | `0.05` | 增强局部轨迹评分对障碍物代价的重视 |

同时确认以下相关参数保持为当前有效值：

```text
acc_lim_theta:          4.0 rad/s²
max_vel_theta:          3.0 rad/s
yaw_goal_tolerance:     0.2 rad
xy_goal_tolerance:      0.1 m
cost_scaling_factor:    3.5
local costmap:          4.0 m × 4.0 m
local costmap resolution: 0.04 m
```

全局膨胀半径现在大于局部膨胀半径，与“全局规划优先留出安全距离、局部规划保留狭窄区域机动性”的配置意图一致。

### 8.3 静态验证结果

2026-07-28 已完成以下检查：

1. `xmllint --noout` 通过，`gazebo_nav.launch` XML 语法正确。
2. `roslaunch --files` 通过，所有启动文件和引用文件可以正常解析。
3. `roslaunch --dump-params` 确认最终生效路径和值为：

```text
/move_base/DWAPlannerROS/acc_lim_theta: 4.0
/move_base/DWAPlannerROS/max_vel_theta: 3.0
/move_base/DWAPlannerROS/min_vel_theta: 0.4
/move_base/DWAPlannerROS/occdist_scale: 0.05
/move_base/DWAPlannerROS/xy_goal_tolerance: 0.1
/move_base/DWAPlannerROS/yaw_goal_tolerance: 0.2
/move_base/global_costmap/inflation_layer/cost_scaling_factor: 3.5
/move_base/global_costmap/inflation_layer/inflation_radius: 0.2
/move_base/local_costmap/inflation_layer/cost_scaling_factor: 3.5
/move_base/local_costmap/inflation_layer/inflation_radius: 0.15
/move_base/local_costmap/width: 4.0
/move_base/local_costmap/height: 4.0
/move_base/local_costmap/resolution: 0.04
```

本轮参数没有语法错误或命名空间覆盖问题，可以进入运行测试。静态检查不能替代 Gazebo 中的动态验证。

### 8.4 运行测试重点

正式比赛前应完整重启导航后测试，不在比赛运行中途修改参数。重点观察：

1. 狭窄通道是否仍能生成局部轨迹，是否出现 `No valid trajectories`、原地振荡或不必要绕行。
2. `occdist_scale=0.05` 是原检查值的 5 倍；若避障反应过强、通道通过率下降，可在下一轮离线调整为 `0.03`，再完整重启测试。
3. 局部 `inflation_radius=0.15 m` 与当前方形 footprint 的内切半径基本相等，软性安全缓冲较小。若出现贴墙或擦碰，应优先测试 `0.18～0.20 m`，而不是继续提高速度。
4. 确认连续 `Off Map` 警告没有复现；局部窗口仍保持 `4.0 m × 4.0 m`。
5. 统计到达目标的成功率、耗时、最小障碍距离以及末端旋转是否平稳，避免只根据单次成功判断参数效果。

### 8.5 赛事规则边界

当前调整符合 `docs/simu_requirement.md` 中允许使用官方规划器原生参数配置文件调试的范围：

- 未修改全局规划器、DWA 或 costmap 源码；
- 未新增二次规划、轨迹拦截或速度干预节点；
- 未读取 `/gazebo/model_states`；
- 未硬编码赛道、障碍物或路径坐标；
- 未对禁止修改的文件进行改动。


---

## 9. 第四轮：修复狭窄区域终点旋转死锁

### 9.1 变更信息

- 修改日期：2026-07-28
- 参数修改文件：`~/gazebo_ws/src/gazebo_nav/launch/config/move_base/dwa_local_planner_params.yaml`
- 记录修改文件：`~/gazebo_ws/docs/COSTMAP参数调整记录.md`
- 未修改 `gazebo_nav.launch`、costmap 配置、URDF、world、地图、Gazebo 插件、规划器源码或其他规则禁止文件。

参数修改完成后的配置文件 SHA-256：

```text
4d98ad8281a03911666baec2e2c0283f3e8d709fceb4d7d13a2d9e0284af5686  gazebo_nav.launch
48d7db04e889eaa5e7d367f82a542c5b7c232526086301e42b1265a7ee40ff5b  dwa_local_planner_params.yaml
24b8530d6d0ad4c8ab2cb70a7dca03f8f0173080c358f1b4ad013580fe2a943f  costmap_common_params.yaml
2d355cf6c74dde92a002b95b957c8e776e0ef4ce5b7d8e395fd45dafa63b19dc  local_costmap_params.yaml
```

### 9.2 两类故障的复现证据

第一类故障是终点原地旋转被局部代价地图判定为碰撞，日志连续输出 `Rotation cmd in collision`。第二类故障不输出该警告，但导航一直保持 `ACTIVE`、没有 `Goal reached`，机器人也不再运动。

在第二类故障的复现中，机器人已经进入位置容忍范围，DWA 持续发布约 `-0.2 rad/s` 的纯旋转命令，但 `/odom` 和 IMU 的实际角速度接近零。把角速度提高到约 `0.3 rad/s` 后，机器人只能短暂转动，随后又进入旋转碰撞判定。因此只调整最大角速度或角加速度会在以下两种状态之间切换，不能消除根因：

1. 角速度较小时，命令不足以克服仿真接触约束，导航保持 `ACTIVE` 但机器人不转；
2. 角速度较大时，机器人开始转动，但狭窄区域中的预测 footprint 很快与障碍栅格冲突，出现 `Rotation cmd in collision`。

该目标要求机器人在狭窄横向通道末端完成接近 `90°` 的原地转向。通道几何空间、机器人外形和安全 footprint 共同决定了这一终点姿态动作缺少稳定余量；膨胀半径不是本轮故障的直接根因。

### 9.3 参数变化

| 参数 | 修改前 | 修改后 | 作用 |
|---|---:|---:|---|
| `sim_time` | `1.5 s` | `1.0 s` | 缩短局部轨迹预测距离，减少狭窄处过早判定候选轨迹不可行 |
| `xy_goal_tolerance` | `0.15 m` | `0.10 m` | 在避免过早锁定位置的同时，按 `0.04 m/格` 的局部地图保留约 2.5 格终点余量 |
| `yaw_goal_tolerance` | `0.2 rad` | `3.2 rad` | 狭窄区域以位置到达为主，跳过缺少空间余量的末端原地旋转 |

以下相关参数保持用户本轮调整值不变：

```text
acc_lim_theta:      2.0 rad/s²
max_vel_theta:      0.6 rad/s
min_vel_theta:      0.1 rad/s
vth_samples:        40
controller_frequency:
  /move_base:                    10.0 Hz
  /move_base/DWAPlannerROS:      10.0 Hz
```

角度误差会被归一化到不超过 `π rad`，因此 `yaw_goal_tolerance=3.2 rad > π` 等价于不再使用最终朝向作为到达条件。该设置会放弃最终朝向精度，这是彻底避免狭窄目标点发生终点旋转死锁的明确取舍；位置精度由 `xy_goal_tolerance=0.10 m` 约束。`0.10 m` 相当于当前局部地图约 2.5 个栅格，比离线边界测试使用的 `0.05 m` 更不容易在最后一两个栅格处停滞。如果任务必须精确满足最终朝向，应把目标点放在有足够旋转空间的位置，或让目标朝向与通道方向一致，不能依靠提高旋转速度在当前狭窄位置安全实现。

### 9.4 运行验证

前两项往返测试使用完整重启后加载的持久化候选参数；第 3～5 项在同一离线实例中用于确定最终容忍度边界；第 6 项是最终 YAML 的完整重启验证。运行数据记录在 `/tmp/gazebo_nav_persisted_clean.bag` 和 `/tmp/gazebo_nav_final_persisted.bag`，ROS 日志位于 `/tmp/codex-nav-repro2/` 及本轮 ROS 日志目录。`/tmp` 文件只用于诊断，不属于正式比赛配置。

1. 狭窄区域目标 `(1.143, -0.112)`、目标角约 `-81°`：机器人实际运动，仿真时间 `106.043 s` 返回 action 状态 `3 / Goal reached`。
2. 从该位置自主返回起点：仿真时间 `117.761 s` 返回 `Goal reached`，最终里程计位置约为 `(0.035, 0.013)`，位置误差约 `0.038 m`。
3. 再次测试此前问题目标 `(1.146, 0.024)`、目标角约 `-91°`：`yaw_goal_tolerance=1.6 rad` 时，机器人停在位置误差约 `0.046 m` 处；到达姿态约 `+7.6°`，角误差约 `1.72 rad`，仍会持续发布 `-0.2 rad/s` 而实际不转。离线临时验证 `1.8 rad` 后，仿真时间 `167.676 s` 明确返回 `Goal reached`，证明放宽末端朝向可以解除死锁。
4. 为覆盖不同接近姿态造成的角误差波动，最终值进一步取 `3.2 rad`。在相同目标位置下给出接近 `180°` 的朝向差，仿真时间 `211.195 s` 仍明确返回 `Goal reached`，未进入原地旋转。
5. 将位置容忍度最终取为 `0.10 m` 后继续执行两次往返目标，仿真时间 `233.100 s` 和 `246.222 s` 均返回 `Goal reached`。
6. 清空所有 ROS/Gazebo 残留后，同时启动 Gazebo 与 RViz，从 YAML 直接加载最终 `xy=0.10、yaw=3.2、sim_time=1.0`。再次执行原问题目标 `(1.146, 0.024)`、目标角约 `-91°`，仿真时间 `103.283 s` 返回 action 状态 `3 / Goal reached`；停止位置约为 `(1.055, -0.011)`，位置误差约 `0.098 m`，符合 `0.10 m` 容忍度。该轮数据保存在 `/tmp/gazebo_nav_final_persisted.bag`。

上述测试日志中：

- 没有出现 `Rotation cmd in collision`；
- 没有出现 `Off Map`；
- 两次出现单帧 `DWA planner failed to produce path`，下一控制周期重新获得全局计划后继续运动，没有形成连续警告、恢复失败或 action 中止；
- 七次导航最终都得到明确的 `Goal reached`，没有遗留 `ACTIVE` 且零运动的目标。

第三至第五项中的动态参数修改只用于离线确定临界值，不是正式运行方案。正式比赛必须在启动前由 YAML 加载固定值，比赛中途不得动态修改参数。

### 9.5 静态验证

2026-07-28 完成：

1. `xmllint --noout` 通过，`gazebo_nav.launch` XML 合法；
2. `roslaunch --dump-params` 确认最终展开值为：

```text
/move_base/DWAPlannerROS/acc_lim_theta: 2.0
/move_base/DWAPlannerROS/max_vel_theta: 0.6
/move_base/DWAPlannerROS/min_vel_theta: 0.1
/move_base/DWAPlannerROS/sim_time: 1.0
/move_base/DWAPlannerROS/xy_goal_tolerance: 0.1
/move_base/DWAPlannerROS/yaw_goal_tolerance: 3.2
/move_base/DWAPlannerROS/controller_frequency: 10.0
/move_base/controller_frequency: 10.0
/move_base/global_costmap/inflation_layer/inflation_radius: 0.15
/move_base/local_costmap/inflation_layer/inflation_radius: 0.15
/move_base/local_costmap/width: 4.0
/move_base/local_costmap/height: 4.0
/move_base/local_costmap/resolution: 0.04
```

### 9.6 正式测试注意事项

本轮是定位参数边界的离线诊断。正式仿真测试仍须按 `docs/simu_requirement.md` 同时启动并显示 Gazebo 与 RViz，并在固定参数下重复覆盖狭窄通道、转弯和终点区域。建议统计多次成功率，而不是只看单次到达；若必须考核最终朝向精度，应先确认目标位置具备足够旋转空间，再单独收紧 `yaw_goal_tolerance`。
