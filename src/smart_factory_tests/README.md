# smart_factory_tests

当前只提供导航阶段的任务发送器，其余测试文件保持占位。

在仿真与导航任务链启动后，发送一个“到达抓取区前置点”的测试任务：

```bash
rosrun smart_factory_tests send_navigation_task.py \
  --task-id nav_demo_001 \
  --target-class food
```

`--target-class` 可取 `food`、`daily` 或 `electronics`。当前里程碑不会执行识别和抓取，
这个字段只是提前验证比赛任务接口能否完整传递。
