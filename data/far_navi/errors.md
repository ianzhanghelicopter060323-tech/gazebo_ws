# far_navi 观察位 OCR 测试结果

测试使用默认参数：`scale=1.0`、`min_text_confidence=0.45`、`class_threshold=0.70`。

## 汇总

| 项目 | 结果 |
|---|---:|
| 图片总数 | 40 |
| 正样本 | 39 |
| 负样本（画面中无物块） | 1 |
| 正样本正确识别 | 27/39（69.23%） |
| 错分类 | 0/39（0%） |
| 漏识为 `unknown` | 12/39（30.77%） |
| 负样本正确拒绝 | 1/1（100%） |

各真实类别的识别结果：

| 真实类别 | 正确/总数 | 正确率 |
|---|---:|---:|
| FOOD | 10/16 | 62.50% |
| DAILY | 5/10 | 50.00% |
| ELECTRONICS | 12/13 | 92.31% |

## 漏识图片

| 图片 | 真实类别 | OCR 输出 | 类别置信度 | OCR 原文 | 原因 |
|---|---|---|---:|---|---|
| [far_daily_good_0001.png](/home/ianichinose/gazebo_ws/data/far_navi/far_daily_good_0001.png) | DAILY | `unknown` | 0 | 空 | 未检测到文字 |
| [far_food_good_0001.png](/home/ianichinose/gazebo_ws/data/far_navi/far_food_good_0001.png) | FOOD | `unknown` | 0.683680 | `食物品快 / 食品 / 物块` | 正面识别正确，但侧面噪声行拉低整体分数 |
| [far_elec_danger_0001.png](/home/ianichinose/gazebo_ws/data/far_navi/far_elec_danger_0001.png) | ELECTRONICS | `unknown` | 0 | `2申` | 物块在最右边缘且严重裁切 |
| [far_daily_good_0004.png](/home/ianichinose/gazebo_ws/data/far_navi/far_daily_good_0004.png) | DAILY | `unknown` | 0 | `日团用` | 类别文字识别错误 |
| [far_food_good_0004.png](/home/ianichinose/gazebo_ws/data/far_navi/far_food_good_0004.png) | FOOD | `unknown` | 0.596680 | `有物 / 食品 / 物块` | 正面识别正确，但侧面噪声行拉低整体分数 |
| [far_food_danger_0001.png](/home/ianichinose/gazebo_ws/data/far_navi/far_food_danger_0001.png) | FOOD | `unknown` | 0 | `食 / 物块` | 物块位于右边缘并裁切，只读到“食” |
| [far_daily_good_0005.png](/home/ianichinose/gazebo_ws/data/far_navi/far_daily_good_0005.png) | DAILY | `unknown` | 0 | `快` | 类别关键词未识别 |
| [far_food_danger_0002.png](/home/ianichinose/gazebo_ws/data/far_navi/far_food_danger_0002.png) | FOOD | `unknown` | 0 | 空 | 物块位于右边缘并裁切，未检测到文字 |
| [far_daily_good_0007.png](/home/ianichinose/gazebo_ws/data/far_navi/far_daily_good_0007.png) | DAILY | `unknown` | 0 | `易快` | 类别文字识别错误 |
| [far_food_good_0012.png](/home/ianichinose/gazebo_ws/data/far_navi/far_food_good_0012.png) | FOOD | `unknown` | 0.517030 | `新 / 食品 / 物块` | 正面识别正确，但侧面噪声行拉低整体分数 |
| [far_daily_good_0010.png](/home/ianichinose/gazebo_ws/data/far_navi/far_daily_good_0010.png) | DAILY | `unknown` | 0 | `易` | 类别关键词未识别 |
| [far_food_good_0014.png](/home/ianichinose/gazebo_ws/data/far_navi/far_food_good_0014.png) | FOOD | `unknown` | 0 | `食物品` | 字符次序错误，未匹配“食品” |

## 成功样本置信度

所有被接受且分类正确的样本置信度均高于 `0.99`。最低的是：

| 图片 | 真实类别 | OCR 输出 | 类别置信度 |
|---|---|---|---:|
| [far_daily_good_0008.png](/home/ianichinose/gazebo_ws/data/far_navi/far_daily_good_0008.png) | DAILY | `DAILY` | 0.996340 |
| [far_daily_good_0009.png](/home/ianichinose/gazebo_ws/data/far_navi/far_daily_good_0009.png) | DAILY | `DAILY` | 0.996710 |

负样本 [far_negative_danger_0001.png](/home/ianichinose/gazebo_ws/data/far_navi/far_negative_danger_0001.png) 中没有物块。OCR 从车体读到 `99`，但没有错误匹配物块类别，最终正确输出 `unknown`。

## 结论

- `far_navi` 正样本单帧识别率为 `69.23%`，明显低于 `mid` 的 `89.74%` 和文档中的 `95%` 初步目标。
- 右侧裁切确实造成失败，但12张漏识中只有3张属于明显边缘裁切，因此只调整导航朝向不能解决全部问题。
- 3张 FOOD 图片已经正确读出正面“食品 / 物块”，却因侧面噪声文字行降低全局最低置信度而被拒绝；应优先把分类置信度改为按匹配文字行或空间分组计算。
- 其余完整物块上的失败主要是文字检测为空或类别字符识别错误，适合继续验证 ROI 裁剪、输入缩放和多帧投票。
