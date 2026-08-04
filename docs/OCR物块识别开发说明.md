# OCR 物块识别开发说明

本文记录当前 OCR 的调用方式、“机器人运行到物块近前再识别”的实现思路，以及后续需要采集的测试数据。本文只覆盖物块文字识别阶段；深度定位、机械臂抓取和完整任务状态机将在对应模块完成后补充。

## 1. OCR 调用方式

### 1.1 当前能力

OCR 位于 `smart_factory_perception` 包中，当前已经实现：

```text
单张图片或单个物块 ROI
→ RapidOCR 文字检测
→ 文字方向判断
→ PP-OCRv6 文字识别
→ 合并上下两行文字
→ 物块词典匹配
→ 输出 FOOD / DAILY / ELECTRONICS / unknown
```

当前词典映射为：

| 物块文字 | 输出类别 | `ExecuteTask.action` 中的目标类别 |
| --- | --- | --- |
| 食品物块 | `FOOD` | `FOOD = 0` |
| 日用物块、日用品物块 | `DAILY` | `DAILY = 1` |
| 电子物块、电子产品物块 | `ELECTRONICS` | `ELECTRONICS = 2` |

OCR 使用本地 ONNX 模型，不需要在识别阶段联网。模型目录为：

```text
src/smart_factory_perception/models/ocr/
```

运行环境为：

```text
/home/ianichinose/gazebo_ws/.venv/ocr
```

### 1.2 推荐调用方式

打开终端并加载 ROS 工作区：

```bash
cd /home/ianichinose/gazebo_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

识别食品物块原始贴图：

```bash
rosrun smart_factory_perception ocr_image \
  src/car3/models/cube/meshes/Food.png \
  --json
```

识别另外两类物块：

```bash
rosrun smart_factory_perception ocr_image \
  src/car3/models/cube/meshes/Daily_Necessities.png \
  --json

rosrun smart_factory_perception ocr_image \
  src/car3/models/cube/meshes/Electronics.png \
  --json
```

`ocr_image` 是专用启动包装器，会自动使用 `.venv/ocr/bin/python`，因此通过 `rosrun` 调用时不需要手动进入 OCR 虚拟环境。

正常输出示例：

```json
{
  "label": "FOOD",
  "confidence": 0.99986,
  "text": "食品 / 物块",
  "bbox": [0, 0, 125, 127],
  "accepted": true
}
```

主要字段含义：

| 字段 | 含义 |
| --- | --- |
| `label` | 物块业务类别；不确定时为 `unknown` |
| `confidence` | 本次类别判断分数，不等同于严格的正确概率 |
| `text` | OCR 识别到的原始文字行 |
| `bbox` | 包含有效文字的图像区域，格式为 `[x, y, width, height]` |
| `accepted` | 是否达到当前分类接受阈值 |

### 1.3 单指令保存相机图片

工作区提供 `imgsave` 命令，每次运行订阅 `/camera/rgb/image_raw`，默认丢弃新订阅后的前 4 帧并保存第 5 帧，随后退出。这可以避免 Gazebo 相机按需启用时保存到姿态改变前的缓存画面，也不需要常驻 `image_saver` 或另开终端调用服务。文件名中的 `%04i`、`%06i` 等整数占位符会自动替换为下一个未占用编号，已有图片不会被覆盖。

```bash
source /home/ianichinose/gazebo_ws/devel/setup.bash
rosrun smart_factory_perception imgsave \
  /home/ianichinose/gazebo_ws/data/close_navi/close_%04i.png
```

重复执行后依次生成：

```text
close_0000.png
close_0001.png
close_0002.png
```

如希望使用更短的命令，可以在当前终端定义：

```bash
alias imgsave='rosrun smart_factory_perception imgsave'
```

之后直接执行：

```bash
imgsave /home/ianichinose/gazebo_ws/data/close_navi/close_%04i.png
```

默认等待相机帧 5 秒。必要时可以通过 `--topic` 和 `--timeout` 修改，例如：

```bash
imgsave --timeout 10 /home/ianichinose/gazebo_ws/data/close_navi/close_%04i.png
```

### 1.4 手动进入和退出虚拟环境

直接运行或调试 OCR Python 代码时，可以手动进入环境：

```bash
cd /home/ianichinose/gazebo_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source .venv/ocr/bin/activate
```

确认当前环境：

```bash
echo "$VIRTUAL_ENV"
which python3
```

应分别看到：

```text
/home/ianichinose/gazebo_ws/.venv/ocr
/home/ianichinose/gazebo_ws/.venv/ocr/bin/python3
```

退出环境：

```bash
deactivate
```

虚拟环境用于隔离 RapidOCR、ONNX Runtime、OpenCV 和 NumPy，避免覆盖 ROS Noetic 的系统 Python 依赖。普通导航和任务节点不需要进入 OCR 环境。

### 1.5 当前验证边界

当前验证使用的是 Gazebo 模型的三张原始贴图：

```text
src/car3/models/cube/meshes/Food.png
src/car3/models/cube/meshes/Daily_Necessities.png
src/car3/models/cube/meshes/Electronics.png
```

三张图片都可以正确输出对应类别，但它们是干净、正视、无背景的纹理源文件，只能证明 OCR 安装、模型推理和词典映射链路正常，不能作为实际相机识别准确率。

## 2. 机器人运行到物块近前识别

### 2.1 基本假设

`spawn_cubes.py` 会将三个物块随机分配到三个已知候选区域。每个区域的大致位置和物块朝向已知，但存在以下随机性：

- 每个区域出现的物块类别不固定；
- 物块在区域内部的精确 `x、y` 坐标随机；
- 每轮重新生成场景后，类别与区域的对应关系会变化。

因此不能使用“区域 A 永远是食品”这样的固定映射。任务中的 `target_class` 决定需要寻找的类别，`task_id` 只负责标识和跟踪本次任务。

### 2.2 为什么要靠近后识别

物块实际尺寸约为 4 cm。距离较远时，物块和汉字在 640×480 图像中占用的像素很少，笔画可能粘连或消失。简单放大低分辨率图像不能恢复已经丢失的笔画。

机器人主动靠近并停稳，可以同时改善：

- 物块在图像中的像素尺寸；
- 汉字笔画完整度；
- OCR 文字检测成功率；
- 字符识别置信度；
- 深度图目标区域的有效像素数量；
- 后续三维定位精度。

### 2.3 推荐任务流程

```text
接收 task_id 和 target_class
→ 完成现有导航，进入抓取区
→ 前往三个候选区域中心的外心观察位，并先朝向 close_navi
→ 小车停稳，机械臂进入相机扫描姿态
→ 判断物块在画面中是否足够大、是否基本正对相机
→ 裁剪物块候选 ROI 并连续执行 OCR
→ 类别稳定且等于 target_class：进入深度定位与抓取
→ 类别稳定但不是目标：保持位置不变，依次转向 mid、far_navi
→ 结果不稳定或为 unknown：调整观察距离/姿态后重试
→ 三个区域都未找到目标：返回识别失败
```

当前首先验证“一个外心位置、三个固定朝向”的方案。若测试表明主要失败原因是物块位于区域边缘、文字被画面裁切或观察距离不足，则将方案升级为三个分别手动微调的位置和朝向，每个位置只负责一个候选区域。若物块已经完整、清晰地位于画面内但仍识别失败，应优先检查 OCR 预处理、阈值或模型，而不是继续增加导航点。

### 2.4 识别确认条件

第一版建议采用保守判定：

- 小车必须停稳后再使用识别结果；
- 一次只处理一个物块 ROI；
- OCR 原始文字必须包含类别关键词，不能只识别到“物块”；
- 最近 5 个有效处理帧中，至少 3 帧输出同一类别；
- 三维位置或图像框中心必须相互接近，避免把不同物块的结果混在一起；
- 稳定类别必须等于任务的 `target_class`；
- 低置信度、类别冲突和过期结果统一视为 `unknown`；
- `unknown` 应触发重新观察，不应强制选择最高分分类。

### 2.5 观察距离的确定方法

当前不再单独为 15、20、25、30、40、50 cm 等每个距离大量采样，而是优先使用正式外心观察位，在每次 Gazebo 随机生成后记录实际观察距离、物块像素宽度和识别结果。这批数据更接近真实比赛分布。若失败明显集中在较远位置或物块像素过小，再补充少量针对性距离样本，最终得到类似下面的运行约束：

```text
当物块宽度 >= N 像素且角度偏差 <= A 度时，才允许确认 OCR 结果。
```

`N` 和 `A` 必须由 Gazebo 相机数据测试确定。观察点只负责让物块进入可靠识别范围，物块在候选区域内的精确位置仍需通过图像和深度数据定位。

### 2.6 OCR 与抓取模块的边界

OCR 只回答“目标是什么”和“文字位于图像哪里”。完整抓取还需要：

```text
OCR 文字框中心 (u, v)
→ 在同步深度图中读取目标深度 Z
→ 根据 CameraInfo 反投影到相机三维坐标 (X, Y, Z)
→ 使用带图像时间戳的 TF 转换到 arm_base_link/base_footprint
→ 机械臂预抓取、微调、闭合夹爪
→ /grasp_attach/state == GRASPING 后确认成功
```

正式运行时不得使用 `/gazebo/model_states` 或 `/gazebo/get_model_state` 代替视觉识别和定位。Gazebo 真值只允许在离线测试工具中作为自动标注和误差评估依据，不能进入比赛任务决策链。

## 3. 需要的测试集

### 3.1 数据集划分原则

数据建议划分为：

- 冒烟样本：快速确认模型、环境和词典能运行；
- 正常随机样本：使用正式固定观察位和角度，验证每次随机生成后的实际识别表现；
- 困难样本：专门检查图像边缘、局部裁切、遮挡和不利生成位置；
- 负样本：检查没有目标文字时是否能够稳定输出 `unknown`。

当前采样单位是“一次独立 Gazebo 启动和随机生成”，而不是视频中的单帧。每次启动后保持同一个观察位置和标称角度，待机器人与画面稳定后保存一张代表图片；如同一次启动保存多张相邻帧，它们仍只能算同一组，不能当成多个独立场景。OCR 输出的 `98%～99%` 是模型置信度，不等同于真实正确率，仍需逐张核对真实类别、最终分类以及是否完整入镜。

如采集过程中调整了 ROI、缩放、置信度阈值或词典，应记录调整点；调整后的新参数至少要用未参与调参的后续独立启动样本重新检查，不能用已经反复查看的图片作为最终结论。

### 3.2 测试集需求表

| 数据组 | 类别/内容 | 建议数量 | 需要覆盖的条件 | 主要用途 | 是否允许调参 |
| --- | --- | ---: | --- | --- | --- |
| 原始贴图冒烟样本 | FOOD、DAILY、ELECTRONICS | 3 张 | 原始 128×128 纹理 | 验证模型加载、字符串输出和词典映射 | 否，不用于准确率结论 |
| 正常随机生成样本 | FOOD、DAILY、ELECTRONICS | 共 60 张，建议每类 20 张 | 60 次独立随机生成；固定外心位置和对应标称朝向；覆盖区域内不同 `x、y` | 判断当前单位置三朝向方案在真实比赛分布下是否可靠 | 可以，但调整参数后需用后续新样本复查 |
| 困难样本 | FOOD、DAILY、ELECTRONICS | 共 15～30 张，建议每类 5～10 张 | 物块靠近画面边缘、局部裁切、轻微遮挡或处于区域不利位置 | 判断失败是否来自观察位覆盖不足，并决定是否改为三个独立识别位置 | 可以，需单独统计 |
| 负样本 | 无目标物块、地板、机械臂、墙面等 | 共 15～20 张 | 画面内不存在有效类别文字，包含可能产生误检的背景 | 检查是否稳定输出 `unknown`，避免抓错物块 | 可以，需单独统计 |

当前阶段的采集目标因此为：`60` 张正常样本、`15～30` 张困难样本和 `15～20` 张负样本。不要为了达到数量而从同一次静止画面连续抽帧；不同随机启动所带来的位置和类别变化比相邻帧数量更有价值。

完成第一轮统计后按失败类型选择方案：

- 正常样本稳定、困难样本只偶发失败：保留一个外心位置和三个朝向，并在失败时增加小角度重试；
- 错误集中在画面边缘、局部裁切或物块尺寸不足：把一个外心位置升级为 close_navi、mid、far_navi 三个独立识别位置，分别手动微调位置和朝向；
- 物块完整清晰入镜仍出现错分类：保留导航方案，优先调整 OCR 或补充针对性训练数据。

### 3.3 每条样本需要保存的信息

静态图片样本至少保存：

| 字段 | 示例 | 用途 |
| --- | --- | --- |
| 图片路径 | `FOOD/run_001/frame_003.jpg` | 定位原始输入 |
| 正确类别 | `FOOD` | 计算分类结果 |
| 是否负样本 | `false` | 统计误报率 |
| 随机种子/运行编号 | `seed_20260803_01` | 防止开发集和测试集泄漏 |
| 候选区域编号 | `A` | 检查区域覆盖情况 |
| 观察距离 | `0.25 m` | 确定可靠距离 |
| 观察角度 | `15°` | 确定允许姿态范围 |
| 物块像素框 | `[x, y, w, h]` | 统计最低像素尺寸 |
| OCR 原始文字 | `食品 / 物块` | 分析检测错误与识别错误 |
| OCR 置信度 | `0.94` | 调整阈值 |
| 最终分类 | `FOOD` | 统计正确、漏识和错分类 |
| 单帧推理耗时 | `85 ms` | 选择处理频率 |

RGB-D 联调时建议优先录制 rosbag，至少包含：

```text
/camera/rgb/image_raw
/camera/rgb/camera_info
/depth_camera/depth/image_raw
/depth_camera/depth/camera_info
/tf
/tf_static
```

开发阶段可以额外记录 Gazebo 真值作为评估标签，但真值话题或服务不得成为正式识别节点的输入。

### 3.4 自动导航采集脚本

第二轮 close_navi 数据使用三个独立识别点中的 seq 35。运行脚本前，在
`src/smart_factory_mission/config/pickup_staging_dev.yaml` 中填写 seq 35 的
`x、y、yaw`。如果修改了位置并仍需朝向 close_navi 刷新区域中心
`(-0.860, -0.525)`，应同时重新计算朝向：

```python
yaw = atan2(-0.525 - y, -0.860 - x)
```

正式采集40张：

```bash
cd /home/ianichinose/gazebo_ws
python3 script/capture_close_navi_images.py
```

固定保存位置为：

```text
/home/ianichinose/gazebo_ws/data/close_navi_second_try/close_auto_%04i.png
```

该入口每轮执行以下流程：

```text
读取最新 pickup_staging_dev.yaml
→ 自动生成只截止到 seq 35 的隔离目标配置和拟合路径
→ 启动全新 Gazebo 并随机刷新物块
→ 沿截断路径导航到 seq 35 的精确位姿
→ 不继续前往 seq 36/37，也不发送旧外心点的二次导航目标
→ 机械臂进入相机扫描姿态
→ 丢弃相机预热帧并保存一张新图
→ 关闭本轮仿真并重新启动
→ 直到目录中存在40张匹配图片
```

修改 seq 35 后，建议先用新目录做一次单张端到端验证：

```bash
cd /home/ianichinose/gazebo_ws

python3 script/capture_pickup_dataset.py \
  --view close_navi \
  --route-end-seq 35 \
  --skip-view-alignment \
  --count 1 \
  --max-attempts 1 \
  --output-format /home/ianichinose/gazebo_ws/data/close_navi_test/close_test_%04i.png
```

显示 Gazebo GUI 进行人工观察时，在单张验证命令末尾增加：

```bash
--gui
```

使用时注意：

- 脚本按输出格式下已有图片总数判断是否完成；目录中已经达到目标数量时会直接退出。因此每次验证新坐标应使用新的测试目录或文件名前缀，不要覆盖旧数据。
- 每次启动都会重新读取 seq 35 并重新拟合，不复用上一次生成的临时路径；拟合路径进入占用或未知栅格时会在启动仿真前报错停止。
- 脚本要求当前没有其他 ROS/Gazebo 会话；检测到外部 ROS master 时会拒绝运行，也不会主动停止外部会话。
- 自动采集的临时目标配置、拟合路径、路径预览图和每轮日志保存在 `script/logs/<启动时间>/`。
- `capture_close_navi_images.py` 固定采集40张到第二轮目录；需要改变数量或输出位置时，应直接调用公共入口 `capture_pickup_dataset.py`。

### 3.5 手动采集命令与建议验收指标

采集图像指令为：

```bash

  rosrun image_view image_saver \
    image:=/camera/rgb/image_raw \
    _save_all_image:=false \
    _encoding:=bgr8 \
    _filename_format:=/home/ianichinose/gazebo_ws/data/close_navi/close_%06i.png \
    __name:=close_navi_saver
```

保持运行，另一终端

```bash
source /home/ianichinose/gazebo_ws/devel/setup.bash
  rosservice call /close_navi_saver/save "{}"
```

一次保存一张

或可用封装的指令

  先加载环境并定义短命令：

```bash
source /home/ianichinose/gazebo_ws/devel/setup.bash
 alias imgsave='rosrun smart_factory_perception imgsave'
```

  之后每次保存只需：

```bash
imgsave /home/ianichinose/gazebo_ws/data/close_navi/close_%04i.png
```

  重复执行会自动生成：

  ```
  close_0000.png
  close_0001.png
  close_0002.png
  ```

  已有文件不会被覆盖。默认订阅``` /camera/rgb/image_raw ```，等待超时为 5 秒。

初步验收目标建议设为：

| 指标 | 建议目标 |
| --- | ---: |
| 独立正样本稳定分类准确率 | ≥ 95% |
| 独立正样本错分类率 | ≤ 1% |
| 独立负样本误报率 | ≤ 1% |
| 可靠观察范围内 `unknown` 比例 | ≤ 5% |
| 目标类别连续 3/5 帧确认成功率 | ≥ 98% |
| OCR 失败后的自主重新观察 | 100% 进入重试或安全失败，不抓错物块 |

对于抓取任务，错分类比暂时返回 `unknown` 更危险。因此阈值和投票策略应优先降低错分类率；漏识可以通过靠近、调整姿态和有限次数重试解决。
