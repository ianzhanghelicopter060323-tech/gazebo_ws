# seq35 单次观察位导航

`run_seq35_observation_once.py` 每次只启动一轮，按正式位置和朝向
约束沿拟合路径导航到 seq35，并在识别、抓取与锥桶区之前停止。
默认显示 Gazebo/RViz 和六个橘色标定方块，到达后保持仿真，按
Ctrl+C 关闭：

```bash
python3 script/run_seq35_observation_once.py
```

如果只需自动验证一轮并在到达后立即关闭：

```bash
python3 script/run_seq35_observation_once.py --headless --exit-after-arrival
```

# 拾取区观察点数据集自动采集

脚本按下面的顺序重复运行，直到输出目录中存在 40 张匹配图片：

```text
启动全新 Gazebo、move_base、Navigate Action Server 和任务服务器
→ 等待机器人、三个随机物块、相机、AMCL、导航 Action 和任务服务器就绪
→ 额外等待仿真稳定
→ 发布现有导航任务并等待 ARRIVED_PICKUP_STAGING
→ 保持外心坐标不变，用 move_base 对准所选区域的绝对朝向
→ 将机械臂移动到已记录的相机扫描姿态 `[0.0, 0.0, 0.55, 2.0, 0.0]`
→ 等待画面稳定并保存一张 RGB 图片
→ 关闭本轮由脚本启动的全部进程
→ 重新启动，获得下一组随机物块位置
```

运行前先关闭其他 ROS/Gazebo 会话。三个图片保存脚本分别为：

```bash
cd /home/ianichinose/gazebo_ws
python3 script/capture_close_navi_images.py
python3 script/capture_mid_images.py
python3 script/capture_far_navi_images.py
```

每次只运行其中一个脚本；不要同时启动三者。

当前 `capture_close_navi_images.py` 用于第二轮 close 数据集：它会从主路线
自动生成只到 seq 35 的隔离目标配置和拟合路径，停在 seq 35 的精确位姿后
直接放下机械臂并拍照，不会继续导航到 seq 36/37，也不会发送旧外心点的
二次对准目标。固定采集 40 张到：

```text
data/close_navi_second_try/close_auto_%04i.png
```

当前 `capture_far_navi_images.py` 用于第三轮 far_navi 数据集：它会自动生成
截止到 seq 37 的路径，使用 seq 37 的精确位置和朝向，不发送旧外心点的
二次导航目标；随后将机械臂移动到文档记录的观察位姿
`[0.0, 0.0, 0.55, 2.0, 0.0]`。当前会在第三轮目录已有40张的基础上
继续采集40张：

```text
data/navi_far_tri_try/far_auto_%04i.png
```

当前 `capture_mid_images.py` 在脚本内独立配置 seq 36 终点
`(-1.395, -0.320, 1.5691910264536908)`，并使用机械臂观察位姿
`[0.0, 0.0, 0.55, 2.10, 0.0]`。脚本每次启动时在本次日志目录内
生成仅到 seq 36 的临时目标配置和拟合路径，不修改工作区中的
`src/smart_factory_mission/config/pickup_staging_dev.yaml` 或
`src/smart_factory_navigation/config/pickup_staging_fitted_path.yaml`。图片固定保存到
`data/mid_second_try/mid_auto_%04i.png`。

默认行为：

- far 当前目标总数为 80 张，即在已有40张编号图片后再采40张；
- 第二轮 close 使用 seq 35；mid 使用私有 seq 36；第三轮 far 使用 seq 37；三者均不发送旧外心对准目标；
- 默认图片分别为 `data/close_navi_second_try/close_auto_%04i.png`、`data/mid_second_try/mid_auto_%04i.png`、`data/navi_far_tri_try/far_auto_%04i.png`；
- Gazebo：无 GUI，RViz 不启动；
- 单轮导航失败或抓图失败：关闭该轮仿真并重新随机启动；
- 每轮退出后会核对并清理本轮 ROS run_id 下的进程；ROS master 未完全退出时立即终止，不复用异常仿真；
- 拍照前重新检查 Gazebo模型、控制器、`/clock` 和相机新帧；近灰度空场景不会写入数据集；
- 机械臂扫描姿态只用于数据集脚本，未加入正式任务状态机；如需禁用可传入 `--skip-arm-pose`；
- 最多启动 80 轮，避免异常状态下无限循环；
- 只统计严格符合 `far_auto_四位数字.png` 的编号图片，`*_fix.png` 等对照图不计数；脚本可以中断后继续；
- 详细日志：`script/logs/<启动时间>/attempt_NNN.log`。

显示 Gazebo GUI：

```bash
python3 script/capture_mid_images.py --gui
```

常用参数：

```bash
python3 script/capture_mid_images.py \
  --count 40 \
  --output-format /home/ianichinose/gazebo_ws/data/mid/mid_auto_%04i.png \
  --startup-settle 8 \
  --photo-settle 1 \
  --navigation-timeout 300 \
  --max-attempts 80
```

按 `Ctrl+C` 可以安全停止；脚本只向自己启动的进程组发送退出信号，不会主动使用 `killall` 或 `pkill`。

## 40 轮端到端锥桶测试

`run_end_to_end_cone_trials.py` 默认执行 40 轮隔离测试。每轮重新启动完整仿真，
由未修改的 `car3/scripts/spawn_cubes.py` 随机生成物块和 10 个锥桶，再从
`food`、`daily`、`electronics` 中随机选择一个完整抓取与配送任务：

```bash
cd /home/ianichinose/gazebo_ws
python3 script/run_end_to_end_cone_trials.py
```

运行前需要关闭其他 ROS/Gazebo 会话。若只需核对随机任务顺序和配置，不启动仿真：

```bash
python3 script/run_end_to_end_cone_trials.py --dry-run --seed 20260810
```

默认输出目录为
`script/logs/end_to_end_cone_trials_<时间>/`，其中包含：

- `trials.csv`：逐轮任务结果、前序/配送导航失败分类和锥桶碰撞摘要；
- `summary.json`：完整汇总及每轮记录；
- `collision_templates.json`：所有碰撞轮的锥桶初始分布；
- `round_NNN/cone_trial.json`：任务阶段、`move_base` 目标和状态、锥桶初末位姿、物理接触与位移证据；
- `round_NNN/stress_template.json`：仅在该轮检测到碰撞时生成，供后续固定分布压力测试使用；
- `round_NNN/roslaunch.log` 和 `cone_monitor.log`：该轮诊断日志。

默认复用 200 轮测试已验证的 Gazebo 服务端世界状态录制方式，同时通过正常仿真
入口启动唯一一个 Gazebo GUI；不会为了录像额外启动第二个 `gzclient`。GUI 继承
`gazebo_nav.launch` 的 `GALLIUM_DRIVER=d3d12`、
`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 和 `LIBGL_ALWAYS_SOFTWARE=0`，默认使用
WSLg 的 NVIDIA 硬件加速。每轮在任务首次进入
`NAVIGATE_TO_PICKUP_STAGING`（开始前序导航）时开始记录，任务结束后封装为
可由 Gazebo 回放的状态日志，单独保存在：

```text
/home/ianichinose/gazebo_ws/data/cone_zone/end_to_end_test/
  end_to_end_cone_trials_<时间>_seed<随机种子>/
    round_NNN/gazebo_world_state.log
    round_NNN/gazebo_world_recording.json
    round_NNN/gazebo_world_recorder.log
```

录制辅助程序仅订阅 `/sim_task/state` 以确定起点，并使用 Gazebo 原生日志控制话题；
不发布任务、导航或车辆控制消息。测试默认显示硬件加速 Gazebo 窗口；无需观察时可
加 `--headless`，服务端录制仍正常进行。禁用录制可加
`--disable-gazebo-recording`。

回放某轮记录：

```bash
python3 script/play_gazebo_world_log.py \
  data/cone_zone/end_to_end_test/<运行目录>/round_NNN/gazebo_world_state.log
```

碰撞默认采用两类只读证据的并集：Gazebo 物理接触流中的
`car3` 与 `cone_*` 接触，以及任务开始基线后锥桶超过 `1 cm` 的平移或
超过 `5°` 的倾倒。监测数据不会发布回任务节点，也不会改变规划器参数、目标或速度。
如物理接触流在当前机器上带来明显额外负载，可用
`--disable-contact-stream` 只保留每 0.05 秒采样一次的锥桶位移/倾倒检测。

碰撞模板只记录后续测试需要的精确位姿和来源。固定分布压力测试应在正常随机生成脚本
运行完后，由独立测试辅助脚本在 Gazebo 中重置锥桶位姿；不要修改
`spawn_cubes.py`，也不要把 Gazebo 真值反馈给导航逻辑。

## Gazebo 录制监测与清理

### 100 轮锥桶区前端到端可靠性测试

`run_pre_cone_e2e_trials.py` 每轮启动全新的完整仿真，由原始
`car3/scripts/spawn_cubes.py` 重新随机分配三个物块的区域和区域内坐标，任务类别也
按记录下来的随机种子独立随机选择。测试覆盖起点导航、物块识别、候选点切换、视觉
对齐和抓取，生成的私有任务配置固定在 `OBJECT_GRASPED` 结束，因此不会进入锥桶区：

```bash
cd /home/ianichinose/gazebo_ws
python3 script/run_pre_cone_e2e_trials.py
```

默认执行 100 轮、无 Gazebo GUI，并从任务提交前开始录制可回放的 Gazebo 世界状态。
录像及镜像的 `trials.csv`、`summary.json` 保存到：

```text
/home/ianichinose/gazebo_ws/data/teb_pre_cone/
  pre_cone_e2e_trials_<时间>_seed<随机种子>/
    round_NNN/gazebo_world_state.log
    round_NNN/gazebo_world_recording.json
    round_NNN/gazebo_world_recorder.log
```

每轮还会在日志目录保存三个物块的真实随机位置 `cube_scene.json`。默认把连续 90 秒
没有任务阶段或细节进展、导航超时/中止以及对齐失败标记为 `stuck=True`；识别结果会
与 Gazebo 中目标物块所在的 35/36/37 号位置核对。只检查配置与随机任务序列可运行：

```bash
python3 script/run_pre_cone_e2e_trials.py --dry-run --seed 20260815
```

测试启动并打印本轮目录名后，可另开终端只监测该次运行，持续删除既没有卡住、也没有
识别失效的轮次录像：

```bash
python3 script/prune_successful_gazebo_recordings.py \
  --watch --apply \
  --run pre_cone_e2e_trials_<时间>_seed<随机种子>
```

锥桶区前测试的清理采用失败关闭策略：只有 `stuck=False`、
`recognition_failure=False`、随机物块场景已核验且录像清单完整一致时才删除。任一判据
缺失或未知都会保留录像。该策略只清理三个 Gazebo 录像文件，不删除逐轮日志、
`cube_scene.json`、`trials.csv` 或汇总文件。

`prune_successful_gazebo_recordings.py` 会同时监测原有 seq35 世界状态录制目录和
上述锥桶测试录像目录。默认只预览，不删除：

```bash
python3 script/prune_successful_gazebo_recordings.py
```

测试期间持续监测并实际清理：

```bash
python3 script/prune_successful_gazebo_recordings.py --watch --apply
```

锥桶录像仅在 `trials.csv` 和录制清单共同证明整轮无错误时删除：任务完成且
`error_code=0`，前序/配送导航均未失败，未检测到锥桶碰撞，锥桶/contact 监测完整，
并且状态日志、任务 ID、路径和文件大小一致。任何字段缺失、未知、异常或不一致都
保留录像。`--apply` 是永久删除；每次删除会在录像根目录生成 JSON 审计报告。
