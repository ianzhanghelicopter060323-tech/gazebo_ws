# TEB 锥桶区端到端压力测试

本目录保存 `TEB_test` 分支的 Gazebo world-state 录像。压力测试固定复现
007、009、012、032 四个锥桶场景，默认轮询运行 20 轮，即每个场景 5 次。
测试仍从完整任务起点开始，避免只测锥桶区而漏掉前序导航退化。

## 运行流程

1. 先确认没有手动启动的 `roscore`、Gazebo 或整套任务。脚本会拒绝复用已有
   ROS master，防止两个 Gazebo 同时运行导致仿真时钟或控制异常。
2. 先执行 dry-run，确认 20 轮顺序为 `007/009/012/032` 循环五次。
3. 运行压力测试。每个逻辑轮次会单独启动仿真、恢复固定锥桶布局、执行完整
   抓取和配送任务、监视锥桶碰撞，并在下一轮前关闭本轮仿真。
4. 查看 `script/logs/fixed_cone_e2e_stress_*` 下的 `summary.json`、
   `trials.csv` 和各轮日志。录像保存在本目录下同名运行目录中。

## 命令

在工作区根目录运行：

```bash
source devel/setup.bash
python3 script/run_fixed_cone_e2e_stress_trials.py \
  --dry-run \
  --experiment-label teb_dijkstra_ginf_0p20_gscale_15p0_nc66
```

无界面的正式压力测试：

```bash
source devel/setup.bash
python3 script/run_fixed_cone_e2e_stress_trials.py \
  --headless \
  --experiment-label teb_dijkstra_ginf_0p20_gscale_15p0_nc66
```

需要观察 Gazebo 时将 `--headless` 改成 `--gui`。脚本默认整任务超时 8 分钟，
连续 6 分钟没有新的任务阶段或路径点进展时判定卡死并终止该轮。

`--experiment-label` 中已识别的参数会与 ROS 参数服务器中的实际值核对，避免
目录名与真正加载的配置不一致。当前示例表示 TEB + Dijkstra、全局膨胀半径
0.20、全局 cost scaling 15.0、neutral cost 66。每轮还会单独保存完整的 TEB、
GlobalPlanner、global/local costmap 有效参数。

如只想复现一个场景，可覆盖默认选择，例如：

```bash
python3 script/run_fixed_cone_e2e_stress_trials.py \
  --source-round 12 --rounds 5 --gui \
  --experiment-label teb_dijkstra_ginf_0p20_gscale_15p0_nc66
```

## 判定标准

以场景为单位：该场景 5 次实验中只要出现一次或以上碰撞、前序导航失败、
锥桶区导航失败或卡死，就判定该场景仍需调参。四个场景全部完成且均无上述
问题时，`summary.json` 中的 `acceptance_passed` 才为 `true`。

基础设施启动失败会自动重启同一逻辑轮次，默认最多重试 2 次；三次均失败时会
终止整批实验，避免继续产生无效轮次。GUI 模式下每项启动依赖的默认等待时间
为 180 秒，所有 car3/cube 模型通过一次 `/gazebo/model_states` 消息统一检查。
基础设施错误会另行记录，不应与 TEB 导航失败混为一谈。

## 录像保留与删除

清理工具只删除已由结果文件证明完全无异常的成功轮次录像；失败、碰撞、结果
缺失或监视不完整的录像都会保留。先预览：

```bash
python3 script/prune_successful_gazebo_recordings.py
```

确认后删除，或在压力测试期间持续清理成功录像：

```bash
python3 script/prune_successful_gazebo_recordings.py --apply
python3 script/prune_successful_gazebo_recordings.py --watch --apply
```

## 后续调参方向

先按失败类型分层调整，避免同时改变全局路径和局部跟踪：

- 全局路径穿过过窄空隙或偏好不理想：调整 `global_planner_params.yaml` 中的
  `neutral_cost`、`cost_factor`，或 `costmap_common_params.yaml` 中的全局膨胀参数。
- 全局路径合理但 TEB 局部轨迹撞桶、穿禁区或可行性重置：优先调整
  `teb_local_planner_params.yaml` 中的 `min_obstacle_dist`、`inflation_dist`、
  `weight_obstacle`、`weight_viapoint`，并同步核对 TEB polygon 与 local costmap
  footprint。
- 路径合理但运动一卡一卡：查看每轮日志中的 feasibility、oscillation 和
  recovery 记录，再小步调整 TEB 时序/采样与 move_base watchdog 参数。

一次只改变一组相关参数，并为每组配置使用新的 `--experiment-label`。不要在
压力测试运行期间编辑 YAML；每轮保存的 `navigation_parameters.json` 用于确认
实际加载值。
