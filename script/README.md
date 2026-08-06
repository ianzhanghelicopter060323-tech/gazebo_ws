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
`(-1.395, -0.355, 1.5691910264536908)`，并使用机械臂观察位姿
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
