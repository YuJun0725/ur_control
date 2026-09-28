# ur7e_vision

该包使用固定 RGB-D 相机识别 Gazebo 桌面上的红、绿、蓝方块，并计算检测点在
`base_link` 坐标系下的三维坐标。

处理流程：

```text
RGB 图像 → HSV 颜色分割 → 方块轮廓与像素中心
                              +
深度图 → 轮廓内部中值深度 → 相机光学坐标系三维点
                              +
CameraInfo 内参 + TF → base_link 三维坐标
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
| `/color_cube_detector/detections/red/center` | 红色方块 `PointStamped` |
| `/color_cube_detector/detections/green/center` | 绿色方块 `PointStamped` |
| `/color_cube_detector/detections/blue/center` | 蓝色方块 `PointStamped` |
| `/color_cube_detector/debug_image` | 带检测框和三维坐标的图像 |
| `/color_cube_detector/markers` | 可在 RViz 中显示的三维球形标记 |

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

发布坐标对应相机看到的方块表面中心，不是仿真中写死的模型位姿。HSV 阈值、轮廓面积、
有效深度和桌面工作范围均可在 `config/color_cube_detector.yaml` 中调整。
