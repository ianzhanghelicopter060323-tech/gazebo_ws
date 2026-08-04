| 图片 | 真实类别 | OCR 输出 | 置信度 | 原因 |
|---|---|---|---:|---|
| [close_daily_danger_0001.png](/home/ianichinose/gazebo_ws/data/close_navi/close_daily_danger_0001.png) | DAILY | `unknown` | 0 | 未检测到文字 |
| [close_elec_danger_0001.png](/home/ianichinose/gazebo_ws/data/close_navi/close_elec_danger_0001.png) | ELECTRONICS | `unknown` | 0 | 未检测到文字 |
| [close_food_danger_0001.png](/home/ianichinose/gazebo_ws/data/close_navi/close_food_danger_0001.png) | FOOD | `unknown` | 0 | 未检测到文字 |
| [close_food_danger_0002.png](/home/ianichinose/gazebo_ws/data/close_navi/close_food_danger_0002.png) | FOOD | `unknown` | 0 | 只识别出 `品 / 块`，类别关键词不完整 |
- 后续进一步修改方向是向mid方向修改导航点：正对时为基准左移
- 目前先完成后两个点的测试集获取，记录失效图片再决定如何修正导航点
- 目前还使用一点识别