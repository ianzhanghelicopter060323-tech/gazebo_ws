# 讯飞智慧工厂智能车仿真工作空间

本仓库是基于 ROS1 Noetic、Gazebo 和 RViz 的智慧工厂智能车仿真工作空间，包含车辆模型、机械臂与夹爪、地图与导航配置，以及面向竞赛任务的任务接口和导航状态机。

当前已实现的开发里程碑为：

> 接收任务 → 检查定位与导航 → 导航至抓取区前置点 → 返回任务结果

目标识别、物块抓取、运输、放置和外部车辆通信暂未完成，对应 ROS 包保留为后续扩展入口。

## 仿真与竞赛要求

开始修改导航或运行仿真前，请先阅读：

-  导航部分简述.md

仿真测试时 Gazebo 与 RViz 应同时运行并保持可见。导航只能通过赛事允许的规划器及其原生参数配置完成；不得读取 `/gazebo/model_states` 等 Gazebo 真值，也不得通过后台遥控或额外节点改写规划输出。

当前 `smart_factory_mission/config/pickup_staging_dev.yaml` 中的路线是开发阶段配置，不代表最终比赛方案。后续应改为由运行时任务或感知结果提供目标，不应把开发坐标直接作为比赛逻辑的替代品。

## 项目结构

```text
gazebo_ws/
├── src/
│   ├── car3/                         # 车辆、机械臂、夹爪、Gazebo 世界和仿真物块
│   ├── gazebo_map/                   # GMapping 建图、地图文件和建图配置
│   ├── gazebo_nav/                   # map_server、AMCL、move_base、costmap 和 DWA 配置
│   ├── roboticsgroup_gazebo_plugins/ # Gazebo 关节模拟插件
│   ├── smart_factory_interfaces/     # ExecuteTask action 和 TaskState 消息
│   ├── smart_factory_mission/        # 任务状态机和导航阶段实现
│   ├── smart_factory_bringup/        # 统一启动入口
│   ├── smart_factory_tests/          # 导航任务测试客户端
│   ├── smart_factory_bridge/         # 外部任务通信占位包
│   ├── smart_factory_perception/     # 视觉感知占位包
│   └── smart_factory_manipulation/   # 机械臂操作占位包
├── docs/                             # 环境、导航调参和任务链路文档
├── build/                            # catkin 构建输出，不提交
├── devel/                            # catkin 开发空间，不提交
└── install/                          # catkin 安装空间，不提交
```

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- Gazebo、RViz、`move_base`、AMCL、`gmapping`、`map_server` 等 ROS 组件
- 已安装并初始化的 `catkin_make`
- 可用的图形环境；WSLg 用户可参考启动文件中的 Mesa D3D12 配置

依赖安装和 WSL/图形环境说明见 [`docs/虚拟仿真总方案.md`](docs/虚拟仿真总方案.md)。

## 快速开始

首次编译或修改 ROS 包后执行：

```bash
cd /home/ianichinose/gazebo_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

启动当前“导航至抓取区前置点”阶段：

```bash
roslaunch smart_factory_bringup nav_to_pickup_stage.launch
```

该入口默认启动 Gazebo、地图服务器、AMCL、`move_base`、RViz 和任务服务器。等待地图、机器人、激光和 costmap 初始化完成后，在另一个终端发送测试任务：

```bash
cd /home/ianichinose/gazebo_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun smart_factory_tests send_navigation_task.py \
  --task-id nav_demo_001 \
  --target-class food
```

`--target-class` 当前支持 `food`、`daily` 和 `electronics`。重复测试时请更换 `--task-id`，因为任务服务器会记住最近完成的任务结果。

## 启动入口

| 命令 | 用途 |
| --- | --- |
| `roslaunch smart_factory_bringup nav_to_pickup_stage.launch` | 启动仿真、导航、RViz 和当前导航任务阶段 |
| `roslaunch smart_factory_mission mission.launch` | 独立启动完整导航与任务服务器 |
| `roslaunch smart_factory_mission mission.launch start_navigation:=false` | 已有导航栈运行时，仅启动任务服务器 |
| `roslaunch gazebo_nav gazebo_nav.launch` | 单独启动 Gazebo 导航栈 |
| `roslaunch car3 gazebo.launch` | 仅启动车辆和 Gazebo 场景 |
| `roslaunch gazebo_map gmapping.launch` | 启动 GMapping 建图 |

同一时间不要同时启动 `mission.launch` 和 `gazebo_nav.launch`，否则会产生重复的 Gazebo、AMCL、`move_base` 或 RViz 节点。需要复用已有导航栈时，使用 `start_navigation:=false`。

## 配置与地图

- 导航参数：`src/gazebo_nav/launch/config/`
- 当前地图：`src/gazebo_map/maps/math_newest.yaml`
- 任务参数：`src/smart_factory_mission/config/mission.yaml`
- 开发路线：`src/smart_factory_mission/config/pickup_staging_dev.yaml`
- 拟合路径：`src/smart_factory_mission/config/pickup_staging_fitted_path.yaml`
- RViz 配置：`src/rviz_default_config.rviz`

如果需要重新测量路线点，可在 RViz 中使用 `2D Nav Goal`，再查看：

```bash
rostopic echo /move_base_simple/goal
```

## 开发协作

```bash
git pull
git switch -c feature/your-task

# 修改并验证后
git status
git diff --check
git add <相关文件>
git commit -m "描述本次修改"
git push -u origin feature/your-task
```

提交前请确认没有把 `build/`、`devel/`、`install/`、ROS 日志、rosbag、缓存或个人环境配置加入 Git。大型运行产物应使用团队约定的共享方式保存，不要直接提交到普通 Git 历史。

## 后续开发方向

1. `smart_factory_bridge`：接入真实或外部任务来源；
2. `smart_factory_perception`：实现目标识别、深度定位和 TF 转换；
3. `smart_factory_manipulation`：实现机械臂预抓取、抓取、运输和放置；
4. `smart_factory_mission`：将导航阶段扩展为完整比赛状态机；
5. `smart_factory_bringup/full_competition.launch`：在各阶段完成并验证后启用完整流程。
