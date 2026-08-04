# mid 观察位 OCR 测试结果

测试使用默认参数：`scale=1.0`、`min_text_confidence=0.45`、`class_threshold=0.70`。

## 汇总

| 项目 | 结果 |
|---|---:|
| 图片总数 | 40 |
| 正样本 | 39 |
| 负样本（画面中无物块） | 1 |
| 正样本正确识别 | 35/39（89.74%） |
| 错分类 | 0/39（0%） |
| 漏识为 `unknown` | 4/39（10.26%） |
| 负样本正确拒绝 | 1/1（100%） |

各真实类别的识别结果：

| 真实类别 | 正确/总数 | 正确率 |
|---|---:|---:|
| FOOD | 11/11 | 100% |
| DAILY | 13/16 | 81.25% |
| ELECTRONICS | 11/12 | 91.67% |

## 漏识图片

| 图片 | 真实类别 | OCR 输出 | 置信度 | 图像情况 |
|---|---|---|---:|---|
| [mid_daily_good_0005.png](/home/ianichinose/gazebo_ws/data/mid/mid_daily_good_0005.png) | DAILY | `unknown` | 0 | 物块完整、正面文字清晰，检测器未检出文字 |
| [mid_elec_danger_0001.png](/home/ianichinose/gazebo_ws/data/mid/mid_elec_danger_0001.png) | ELECTRONICS | `unknown` | 0 | 物块位于画面最右侧且明显裁切 |
| [mid_daily_good_0007.png](/home/ianichinose/gazebo_ws/data/mid/mid_daily_good_0007.png) | DAILY | `unknown` | 0 | 物块完整、正面文字清晰，检测器未检出文字 |
| [mid_daily_good_0008.png](/home/ianichinose/gazebo_ws/data/mid/mid_daily_good_0008.png) | DAILY | `unknown` | 0 | 物块完整、正面文字清晰，检测器未检出文字 |

## 低置信度图片

本批成功样本中，除下图外的最低类别置信度为 `0.998170`：

| 图片 | 真实类别 | OCR 输出 | 类别置信度 | OCR 原文 | 图像情况 |
|---|---|---|---:|---|---|
| [mid_daily_danger_0001.png](/home/ianichinose/gazebo_ws/data/mid/mid_daily_danger_0001.png) | DAILY | `DAILY` | 0.932950 | `物块 / 日用 / 物块` | 物块靠近右边界并部分裁切，同时识别到侧面和正面文字 |

负样本 [mid_negative_danger_0001.png](/home/ianichinose/gazebo_ws/data/mid/mid_negative_danger_0001.png) 中没有物块，OCR 正确输出 `unknown`，置信度为 `0`。

## 结论

- 当前 `mid` 固定观察位没有错分类，但正样本单帧识别率 `89.74%`，低于文档中的 `95%` 初步目标。
- 只有一个漏识样本是明显的画面边缘裁切；另外三个 `DAILY` 漏识样本完整且清晰，说明问题不应只归因于导航朝向。
- 后续应先检查 OCR 文字检测阶段对这三个完整 `DAILY` 样本的缩放或预处理效果，再决定是否继续修改导航点。
- 目前仍使用一个观察位置识别三个区域；后续先完成其余观察方向的数据获取，再综合失效图片决定是否改成三个独立导航点。
