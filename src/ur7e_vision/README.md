# ur7e_vision

该包使用固定 RGB-D 相机识别 Gazebo 桌面上的红、绿、蓝方块，并利用方块的已知尺寸估计其几何中心与水平朝向，结果位于 `base_link` 坐标系。

处理流程：

```text
RGB 图像 → HSV 颜色分割 → 方块轮廓
                            +
深度图 + CameraInfo → 轮廓内三维点云
                            +
相机 TF → base_link 点云 → 顶面矩形拟合 + 已知高度 → 几何中心与偏航角
```

## 运行

终端 1 启动 Gazebo：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenqin/project/ur_control/install/setup.bash
ros2 launch ur7e_gazebo ur7e_gazebo.launch.py
```

终端 2 启动识别：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenqin/project/ur_control/install/setup.bash
ros2 launch ur7e_vision color_cube_detector.launch.py
```

输出话题：

| 话题 | 内容 |
|---|---|
| `/color_cube_detector/detections/red/center` | 红色方块几何中心 `PointStamped` |
| `/color_cube_detector/detections/green/center` | 绿色方块几何中心 `PointStamped` |
| `/color_cube_detector/detections/blue/center` | 蓝色方块几何中心 `PointStamped` |
| `/color_cube_detector/detections/{red,green,blue}/pose` | 对应方块中心与水平朝向 `PoseStamped` |
| `/color_cube_detector/debug_image` | 带检测框和三维坐标的图像 |
| `/color_cube_detector/markers` | 可在 RViz 中显示的方块模型（中心和朝向来自估计） |

查看坐标：

```bash
ros2 topic echo /color_cube_detector/detections/red/center
ros2 topic echo /color_cube_detector/detections/green/center
ros2 topic echo /color_cube_detector/detections/blue/center
```

查看标注图：

```bash
ros2 run rqt_image_view rqt_image_view /color_cube_detector/debug_image
```

算法只使用 RGB-D、相机内参与 TF，以及 `object_size` 给出的已知物体尺寸；不读取 Gazebo 的模型位姿或桌面高度。顶面点数不足或拟合尺寸明显不符时，不发布位姿。HSV 阈值、轮廓面积、有效深度和工作范围均可在 `config/color_cube_detector.yaml` 中调整。

第一版适用于直立、静止、顶面大部分可见的方块。算法从最高点附近提取顶面点带，拟合水平矩形，其中心 Z 减去已知高度的一半得到几何中心；尚不支持任意倾斜物体的完整六维位姿。正方形顶面的朝向存在 90° 对称性，输出选择接近基座坐标轴的等价偏航角。遮挡和分割误差仍可能使中心偏移，尺寸筛选不能保证排除所有误检。

## 按视觉位姿抓取

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenqin/project/ur_control/install/setup.bash
ros2 launch ur7e_gazebo ur7e_gazebo_demo.launch.py use_vision_target:=true
```

该命令同时启动仿真、MoveIt、RViz、识别和一次红色方块抓取；不要同时运行另一套仿真或识别节点。夹爪会根据估计偏航角调整水平朝向。红、绿、蓝都输出定位结果，当前抓取演示默认选择红色。绿色和蓝色尚未自动加入 MoveIt 的障碍物列表。

抓取任务检查位姿的坐标系、时效和桌面工作范围；静态桌面配置只用于碰撞规划与范围校验，不用于修改相机估计出的中心。视觉模式不会回退到固定目标位置，也不会复位 Gazebo 方块。`object_size` 应与运动任务的 `target_size` 保持一致。

## 本次仿真验收（2026-09-29）

默认相机和方块摆放下，对比 Gazebo 落稳后的几何中心（单位：mm）：

| 方块 | 原表面点三维偏差 | 本版几何中心三维偏差 |
|---|---:|---:|
| 红 | 19.02 | 1.88 |
| 绿 | 18.70 | 2.24 |
| 蓝 | 18.33 | 2.06 |

这是一组静态场景测量，不是所有位置、光照和遮挡情况的精度保证。没有使用手工减去固定 XYZ 偏差的补偿。

已完成两次 Gazebo 物理抓取放置：默认红色方块位置，以及仅在验收时将红色方块移至约 `[0.20, 0.47, 0.33]` 后的位置。第二次任务使用的视觉中心为 `[0.2004, 0.4683, 0.3305]`，放置后实体中心约为 `[0.1627, 0.3237, 0.3300]`。靠近隔断的 `[0.12, 0.44, 0.33]` 测试位置虽然能识别，但预抓取目标被 MoveIt 拒绝，任务按设计停止。
