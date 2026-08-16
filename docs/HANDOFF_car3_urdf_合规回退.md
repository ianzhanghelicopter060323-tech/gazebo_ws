# HANDOFF:car3.urdf 合规回退(模型层修改恢复)

> 交接文档。目的:把 `src/car3/urdf/car3.urdf` 中不符合竞赛规则的模型层修改回退到与标准模型一致,同时保留规则允许的改动。
> 交接日:2026-08-16 | 分支:`TEB_test`

---

## 一、背景与合规判定

竞赛规则(组委会口径):
- world 地图模型、随机生成的 py 文件:**不能改**
- 机械臂、抓取的两个 py 文件:**可以改**
- 车如果摇:**只允许改 urdf 里的惯性参数**
- 模型:**不能修改**

**判定逻辑:** 规则单独为"惯性参数"开口子,说明 URDF 默认落在"模型不能修改"的禁止范围内;能动的只有一条——车摇时改 `<inertial>`(质量/惯量/惯性参考系)。据此,当前工作区对 URDF 的改动中:

- **质心下移(惯性参考系 origin)** → 属惯性参数范畴,**合规,保留**
- **depth 话题配置** → 不改变物理模型,**保留**
- **四轮碰撞体换成圆柱、删除 laser 碰撞体** → 碰撞几何改动,**不合规,回退**
- **深度相机插件 openni_kinect → depth_camera** → 传感器改动,不在惯性参数豁免内,**先研究再决定**

---

## 二、关键文件与基线

| 角色 | 路径 |
|---|---|
| 当前模型(待回退) | `src/car3/urdf/car3.urdf` |
| 标准模型(回退基准) | `/home/ianichinose/gazebo_ws_std/gazebo_ws/src/car3/urdf/car3.urdf` |
| 网格文件 | `src/car3/meshes/*.STL` — 与标准逐字节一致,**无需处理** |

回退的最终验收标准:`diff` 当前文件与标准文件后,仅剩"保留项 + 待研究项 + 行尾换行"三处差异(见第六节)。

---

## 三、决策总表

| # | 改动点 | 当前值 | 标准值 | 决策 |
|---|---|---|---|---|
| 1 | `base_link` 惯性参考系 origin | `-0.005 0 0.035` | `0 0 0.045754` | **保留**(质心下移抗倾倒,合规) |
| 2 | 深度相机话题配置 | `rgb/image_raw`(imageTopicName)+ depth 各话题 | `depth/image_raw`(imageTopicName) | **保留**(depth 话题配置) |
| 3 | 四轮碰撞体(×4) | 圆柱 `radius=0.0489 length=0.052` + `rpy=1.5708 0 0` | mesh `wheel_XX_link.STL` + `rpy=0 0 0` | **回退** |
| 4 | `laser_link` 碰撞体 | 已删除(仅 visual + 注释) | mesh `laser_link.STL` | **回退** |
| 5 | 深度相机插件 | `libgazebo_ros_openni_kinect.so` | `libgazebo_ros_depth_camera.so` | **待研究**(默认暂不回退) |

> ⚠️ **勘误声明(2026-08-16):** 本次核对发现两处与早期口头结论相反的事实,已按事实修正:
> 1. `base_link` 改动的是**惯性参考系 origin(质心位置)**,不是碰撞体。碰撞体 origin 两边都是 `0 0 0`,从未改过。
> 2. 当前工作区深度插件是 **`libgazebo_ros_openni_kinect.so`**(旧),标准是 **`libgazebo_ros_depth_camera.so`**(新)。回退方向 = 当前 openni_kinect → 标准 depth_camera。

---

## 四、保留项(KEEP)详情

### 4.1 `base_link` 质心下移抗倾倒 —— 保留

位置:当前文件第 19 行。

```xml
<inertial>
  <origin xyz="-0.005 0 0.035" rpy="0 0 0" />   <!-- 标准为 0 0 0.045754 -->
  <mass value="20.0" />
  <inertia ixx="0.049" ixy="0" ixz="0" iyy="0.093" iyz="0" izz="0.132" />
</inertial>
```

- 质量(20.0)与惯量张量(0.049/0.093/0.132)与标准**完全一致**,改的只是惯性参考系原点。
- 作用:质心整体下移 ~1.07cm,降低重心、提高抗倾覆稳定性。
- 合规性:`<inertial>` 参数调整,在"车摇可改惯性参数"口子内,**合规,不回退**。
- 注意:质心坐标带 x 偏移 `-0.005`,与抗倾倒无直接关系,若后续想更保守可只保留 z 下移,但当前判定为保留原值。

### 4.2 depth 话题配置 —— 保留

位置:当前文件第 1084-1090 行(深度 sensor 的 plugin 段)。

```xml
<!-- The RGB-D plugin also exposes its RGB render. Keep that image
     separate from the 32FC1 depth image below: publishing both on
     one topic makes image_transport/rqt_image_view unreliable. -->
<imageTopicName>rgb/image_raw</imageTopicName>
<cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
<depthImageTopicName>depth/image_raw</depthImageTopicName>
<depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
<pointCloudTopicName>depth/points</pointCloudTopicName>
```

- 深度话题 `depth/image_raw` / `depth/camera_info`(即 `depthImageTopicName` / `depthImageCameraInfoTopicName`)与标准**一致**,是感知下游实际消费的 topic(见 5.2)。
- 与标准的差异仅在 `imageTopicName` / `cameraInfoTopicName`(当前指向 RGB,标准指向 depth)。**保留当前配置**。

---

## 五、回退项(REVERT)详情

### 5.1 四轮碰撞体:圆柱 → 网格(×4)

位置:当前文件第 57-62 / 86-91 / 115-120 / 144-149 行,四个 `wheel_lf / wheel_rf / wheel_lb / wheel_rb` 的 `<collision>` 块。

**当前(不合规,回退):**

```xml
<collision>
  <origin xyz="0 0 0" rpy="1.5708 0 0" />
  <geometry>
    <cylinder radius="0.0489" length="0.052" />
  </geometry>
</collision>
```

**恢复为(与标准一致):**

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <mesh filename="package://car3/meshes/wheel_XX_link.STL" />
  </geometry>
</collision>
```

其中 `wheel_XX_link` 对应各轮:lf / rf / lb / rb。注意:**只改 collision 块**,visual 与 inertial 不动(visual 本来就引用同一 STL,inertial 与标准一致)。

### 5.2 `laser_link` 碰撞体:恢复

位置:当前文件第 157-181 行。当前把碰撞体删了,留了一段注释(第 172-175 行)。

**删除当前注释:**

```xml
<!--
  Gazebo 射线从 laser_link 原点发出；雷达外观网格若带碰撞体，会产生大量自体回波，
  进而把机器人足迹标成致命障碍。保留 visual，但不为装饰雷达设置 collision。
-->
```

**恢复 collision 块(与标准一致):**

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <mesh filename="package://car3/meshes/laser_link.STL" />
  </geometry>
</collision>
```

> ⚠️ 回退此条的直接后果:激光可能在雷达外观网格上产生自体回波,把本体足迹标成致命障碍。**回退后必须做一轮导航回归**,确认代价地图正常(若复现自反射问题,处理手段应转向导航侧——局部代价地图/传感器配置/路径曲率——而不是改模型,见第七节)。

---

## 六、待研究项(PENDING):深度相机插件

### 6.1 事实

- 当前:`libgazebo_ros_openni_kinect.so`(旧插件)
- 标准:`libgazebo_ros_depth_camera.so`(新插件)
- 回退方向:当前 openni_kinect → 标准 depth_camera。**回退前必须研究对本仿真感知链路的实际影响,再决定是否执行。默认暂不回退。**

### 6.2 下游消费关系(已查)

感知节点 `src/smart_factory_perception/scripts/cube_locator.py`(参数在 `config/cube_locator.yaml`)消费:

| 参数 | topic | 来源 |
|---|---|---|
| `rgb_topic` | `/camera/rgb/image_raw` | 独立 RGB sensor(`libgazebo_ros_camera.so`,两边一致) |
| `depth_topic` | `/depth_camera/depth/image_raw` | 深度 sensor 的 `depthImageTopicName`(两边一致) |
| `camera_info_topic` | `/depth_camera/depth/camera_info` | 深度 sensor 的 `depthImageCameraInfoTopicName`(两边一致) |

即:感知消费的 depth topic 由 `depthImageTopicName` / `depthImageCameraInfoTopicName` 提供,**这两个参数两边完全一致**。插件差异体现在 `imageTopicName`(当前=RGB,标准=depth)与插件自身的输出行为。

### 6.3 研究清单(回退前需确认)

1. **topic 输出差异**:`libgazebo_ros_depth_camera.so` 在 `imageTopicName` / `depthImageTopicName` 都配置时实际发布哪些 topic?是否会按标准配置把"image"变成深度图,从而改变 `/depth_camera/...` 命名空间下的输出?——可用 `rostopic list` 在两个配置下各起一次仿真对比。
2. **RGB 是否受影响**:当前 openni_kinect 会把 RGB 发到 `imageTopicName=rgb/image_raw`,与独立 RGB sensor 同 topic 可能冲突;换成 depth_camera 后该 topic 不再有重复发布者。确认 `cube_locator` 的 `rgb_topic=/camera/rgb/image_raw` 仍由独立 RGB sensor 正常供给。
3. **32FC1 深度格式**:注释提到当前 depth 是 32FC1。确认 depth_camera 插件输出格式与 `cube_locator` 的解析兼容(分辨率 640x480、near/far 不变)。
4. **回归**:换插件后跑一遍 cube 识别/抓取链路,确认深度图能正常出点。

### 6.4 如果决定回退,改动点

当前文件第 1080 行:plugin 换成 `libgazebo_ros_depth_camera.so`,并把第 1084-1088 行的 `imageTopicName` / `cameraInfoTopicName` 按标准改为 `depth/image_raw` / `depth/camera_info`(同时删注释)。**注意:话题配置与插件是耦合的,回退插件时应一并采用标准配置。**

---

## 七、回退后的期望 diff(验收基准)

执行 5.1 + 5.2 之后(4.1/4.2 保留、6.1 暂不动),`diff 当前 标准` 应**只剩**以下差异:

```
19c19                                     ← base_link 惯性参考系 origin(保留项)
< <origin xyz="-0.005 0 0.035" ...>
> <origin xyz="0 0 0.045754" ...>
1080c1082                                  ← 深度插件(待研究项,暂不回退)
< libgazebo_ros_openni_kinect.so
> libgazebo_ros_depth_camera.so
1084,1088c1086,1087                       ← depth 话题配置(保留项)
< 注释 + rgb/image_raw + rgb/camera_info
> depth/image_raw + depth/camera_info
1133c1132                                 ← 行尾换行(无关紧要)
< </robot>
> </robot> (无末尾换行)
```

**如果 diff 里还出现车轮、laser 或其他差异,说明回退没做干净。**

---

## 八、执行与验证步骤

1. 按第五节回退 4 轮碰撞体与 laser 碰撞体,4.1/4.2 保持不动,6.1 先不动。
2. 用标准文件做基准 diff,确认只剩第七节的 4 处差异。
3. 重新编译/加载模型(`catkin_make` 或对应包的 build),启动仿真。
4. **导航回归**:重点验证激光代价地图正常、无自反射导致的本体致命障碍、锥桶区穿越行为不劣化。
5. **感知回归**:确认 `/camera/rgb/image_raw` 与 `/depth_camera/depth/image_raw` 正常出图,`cube_locator` 能出点。
6. 深度插件回退与否,按第六节研究清单跑完对比后**单独决策**,决策结论补记到本文档末尾。

---

## 九、合规提示(留给接手人)

- 回退的方向是**向标准看齐**;如果后续调试中又需要"修车",优先走规则允许的路线:
  - 车摇 → 只改 `<inertial>` 参数(质量/惯量/质心),不碰碰撞几何。
  - 激光自反射/前向障碍伪影 → 导航侧处理(代价地图参数、传感器配置、路径曲率),见 `round064-70pct-escape-repro-report` 记忆与 `docs/round064_70pct_escape_repro_report_20260816.md`。
  - 感知不可用 → 先和组委会确认改传感器插件是否算"模型修改"再动手。
