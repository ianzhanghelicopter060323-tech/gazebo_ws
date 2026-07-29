# AMCL 参数调整与赛事合规记录

## 1. 变更信息

- 规则核对日期：2026-07-27
- 修改日期：2026-07-27
- 修改文件：`~/gazebo_ws/src/gazebo_nav/launch/config/amcl/amcl_omni.launch`
- 修改前备份：`~/gazebo_ws/src/gazebo_nav/launch/config/amcl/amcl_omni.launch.backup-20260727-before-gazebo-odom-tuning`
- 修改前 SHA-256：`9cb0bc45d96def0c6279ecf8564de2ef3516f95ccb7b408f473ef7c066313ae1`
- 修改后 SHA-256：`ea69d43fc3bfe55eb7d12d9acbd608b2d2d102ff6b33e6189d8550501149d93c`
- 调整目的：降低 AMCL 对 Gazebo 精确里程计施加不必要的大范围粒子扩散，减少错误激光匹配造成的 `map -> odom` 突变，同时保留 AMCL 自主定位和激光校正能力。

## 2. 赛事合规结论

截至 2026-07-27，根据当前能够核对的比赛规则和官方 Gazebo 手册，本次修改符合现有书面要求，理由如下：

1. `~/gazebo_ws/docs/simu_requirement.md` 明确允许通过原生参数配置文件调试内置参数；本次只修改 AMCL 原生参数，没有修改或封装赛事规划器源码。
2. 同一要求明确禁止读取 `/gazebo/model_states`，并要求定位依靠激光雷达、视觉或 SLAM 自主解算。本次没有读取 `/gazebo/model_states`、`/gazebo/get_model_state` 等仿真真值接口；AMCL 仍使用 `/scan`、`/odom` 和静态地图进行激光定位。
3. `~/iflytek-smart-car/rules/完整规则.pdf` 的“子任务 3：仿真系统协同”要求 Gazebo 中的虚拟机器人自动执行协同任务，但没有规定 AMCL 必须使用固定参数，也没有禁止调整定位、导航算法配置。
4. `~/gazebo_ws/docs/gazebo仿真操作手册.pdf` 第 4 页明确包含启动 `move_base + AMCL` 的流程。本次仍然使用 AMCL，没有绕过定位模块。
5. 官方手册第 8 页明确标注 `spawn_cubes.py` 不可以修改、比赛时裁判会查看源码。本次没有修改该文件，也没有改变物块或锥桶的数量、位置生成逻辑。
6. `tf_broadcast` 被显式保持为 `true`，`map -> odom` 仍由 AMCL 根据粒子滤波定位结果计算，不是固定或伪造的 TF。
7. 本次没有采用 2026-07-23 测试过的“关闭 GMapping 位姿搜索、单粒子、零运动噪声”方案，也没有发布固定 `map -> odom`。这些更激进的真值锁定方案继续停用。

因此，本次属于对官方定位算法的参数调优，而不是使用 Gazebo 真值替代自主定位。

> 合规边界：若组委会在赛前发布新的规则、技术群澄清或现场裁判要求，应以最新正式解释为准。尤其不得进一步关闭 AMCL TF、固定 `map -> odom`，或通过 Gazebo 服务直接获取机器人/物块真值位置，除非组委会明确允许。

## 3. 修改前数据

- 数据记录日期：2026-07-27
- 数据来源：本次日期备份

| 参数 | 修改前值 |
|---|---:|
| `odom_model_type` | `omni` |
| `odom_alpha1` | `0.2` |
| `odom_alpha2` | `0.2` |
| `odom_alpha3` | `0.8` |
| `odom_alpha4` | `0.2` |
| `odom_alpha5` | `0.1` |
| `laser_max_beams` | `50` |
| `laser_model_type` | `beam` |
| `laser_z_hit` | `0.5` |
| `laser_z_short` | `0.05` |
| `laser_z_max` | `0.05` |
| `laser_z_rand` | `0.5` |
| `laser_sigma_hit` | `0.2` |
| `update_min_d` | `0.2 m` |
| `update_min_a` | `0.5 rad` |
| `base_frame_id` | 未显式设置，使用 AMCL 默认值 `base_link` |
| `global_frame_id` | 未显式设置，使用 AMCL 默认值 `map` |
| `tf_broadcast` | 未显式设置，使用 AMCL 默认值 `true` |
| `initial_pose_x` | `0.0 m` |
| `initial_pose_y` | `0.175 m` |
| `initial_pose_a` | `0.0 rad` |
| `initial_cov_xx` | 未显式设置，使用 AMCL 默认值 |
| `initial_cov_yy` | 未显式设置，使用 AMCL 默认值 |
| `initial_cov_aa` | 未显式设置，使用 AMCL 默认值 |

## 4. 修改后数据

- 生效配置日期：2026-07-27

| 参数 | 修改后值 | 修改原因 |
|---|---:|---|
| `odom_model_type` | `omni` | 保持全向底盘模型 |
| `odom_alpha1` | `0.001` | 减少旋转运动引入的角度噪声 |
| `odom_alpha2` | `0.001` | 减少平移运动引入的角度噪声 |
| `odom_alpha3` | `0.001` | 大幅减少平移粒子扩散 |
| `odom_alpha4` | `0.001` | 减少旋转运动引入的平移噪声 |
| `odom_alpha5` | `0.001` | 减少全向横移噪声，同时保留非零不确定性 |
| `laser_max_beams` | `120` | 使用更多有效激光束降低重复环境误匹配概率 |
| `laser_model_type` | `likelihood_field` | 提高对未写入静态地图的随机锥桶、物块的鲁棒性 |
| `laser_z_hit` | `0.7` | 主要使用地图障碍物匹配 |
| `laser_z_short` | `0.0` | `likelihood_field` 模型不使用短读数分量 |
| `laser_z_max` | `0.0` | `likelihood_field` 模型不使用最大量程分量 |
| `laser_z_rand` | `0.3` | 为随机锥桶、物块和离群读数保留容错 |
| `laser_sigma_hit` | `0.1` | 适配 Gazebo 中较稳定的激光测量 |
| `update_min_d` | `0.05 m` | 更频繁地执行小幅定位更新 |
| `update_min_a` | `0.1 rad` | 避免积累约 0.5 rad 后一次性明显修正 |
| `base_frame_id` | `base_footprint` | 与 Gazebo odom 的机器人子坐标系一致 |
| `global_frame_id` | `map` | 显式固定标准全局定位坐标系 |
| `tf_broadcast` | `true` | 保留 AMCL 自主计算和发布 `map -> odom` |
| `initial_pose_x` | `0.0 m` | 本轮保持不变 |
| `initial_pose_y` | `0.175 m` | 可能是已有地图的实测对齐量，本轮不擅自归零 |
| `initial_pose_a` | `0.0 rad` | 本轮保持不变 |
| `initial_cov_xx` | `0.0004 m²` | 已知起点附近约 `2 cm` 标准差 |
| `initial_cov_yy` | `0.0004 m²` | 已知起点附近约 `2 cm` 标准差 |
| `initial_cov_aa` | `0.0003 rad²` | 约 `1°` 标准差 |

以下粒子滤波机制继续保留：

- `min_particles=1000`
- `max_particles=10000`
- `kld_err=0.05`
- `kld_z=0.99`
- `resample_interval=1`
- `recovery_alpha_slow=0.0`
- `recovery_alpha_fast=0.0`

AMCL 仍然使用多粒子、激光似然和 KLD 自适应采样，不是 odom-only 定位。

## 5. 验证结果

2026-07-27 已完成：

1. 日期备份的 SHA-256 与修改前文件一致，确认备份没有遗漏。
2. `xmllint --noout` 检查通过，修改文件和备份均为合法 XML。
3. `roslaunch --files gazebo_nav gazebo_nav.launch start_gazebo:=false start_rviz:=false` 解析通过，能够正确加载：
   - `gazebo_nav.launch`
   - `amcl_omni.launch`

运行时效果仍需在下一次 Gazebo 导航测试中确认，重点记录：

- `odom -> base_footprint` 是否保持连续；
- `map -> odom` 最大单步平移和旋转；
- 是否还出现 RViz 中的视觉瞬移；
- 随机锥桶和物块存在时，AMCL 粒子云是否保持单峰；
- 机器人能否正确到达各仓库且不碰撞锥桶。

## 6. 回退方法

如需恢复 2026-07-27 修改前配置：

```bash
cp ~/gazebo_ws/src/gazebo_nav/launch/config/amcl/amcl_omni.launch.backup-20260727-before-gazebo-odom-tuning \
   ~/gazebo_ws/src/gazebo_nav/launch/config/amcl/amcl_omni.launch
```

恢复或修改参数后，需要重新启动导航：

```bash
roslaunch gazebo_nav gazebo_nav.launch
```
