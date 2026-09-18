# ur7e_motion

通过 MoveIt 2 和 Mock Hardware，用 Python 向 UR7e 发送单次运动指令。

## 安装依赖

```bash
sudo apt-get install ros-jazzy-moveit-py
```

## 构建

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ur7e_motion
source install/setup.bash
```

## 启动仿真控制链路

终端 1：启动 UR7e、2F-85 Mock Hardware 和 `ros2_control` 控制器。

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenqin/project/ur_control/install/setup.bash
ros2 launch ur7e_description ur7e_mock_control.launch.py
```

终端 2：启动 MoveIt 2 和 RViz。

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenqin/project/ur_control/install/setup.bash
ros2 launch ur7e_moveit_config ur7e_moveit.launch.py
```

## 执行运动

以下命令均在新的终端中执行。每次启动只规划并执行一个目标，完成后节点自动退出。

### 默认关节目标

不传参数时使用默认关节角，单位为弧度：

```bash
source install/setup.bash
ros2 launch ur7e_motion motion_command.launch.py
```

### 自定义关节目标

`joint_positions` 中的 6 个数值依次对应：

1. `shoulder_pan_joint`
2. `shoulder_lift_joint`
3. `elbow_joint`
4. `wrist_1_joint`
5. `wrist_2_joint`
6. `wrist_3_joint`

```bash
source install/setup.bash
ros2 launch ur7e_motion motion_command.launch.py \
  mode:=joint \
  joint_positions:="[1.54, -1.62, 1.4, -1.350796327, -1.570796326589793, -0.030796327]"
```

### 末端位姿偏移

以下命令在保持当前末端姿态的同时，使夹爪中心 `robotiq_tcp` 沿 `base_link` 坐标系的 Z
轴正方向移动 5 cm。`translation` 的顺序为 `[dx, dy, dz]`，单位为米：

```bash
source install/setup.bash
ros2 launch ur7e_motion motion_command.launch.py \
  mode:=pose \
  translation:="[0.0, 0.0, 0.05]"
```

## 参数说明

- `mode`：运动模式，可选 `joint` 或 `pose`，默认为 `joint`。
- `joint_positions`：6 个关节的目标角度，单位为弧度。
- `translation`：末端相对当前位置的平移量，基于 `base_link` 坐标系，单位为米。

只有规划成功时程序才会发送轨迹。参数无效、机器人状态不可用、规划失败或执行失败
时，程序会打印错误原因并返回非零退出码。

## 多候选轨迹选择

每个运动目标会调用 OMPL 生成 3 条候选轨迹。程序只保留通过 MoveIt 碰撞验证的
候选，并按六个关节的累计欧氏角距离计算路径长度，最后只执行其中最短的一条。
规划日志会显示每条候选轨迹的长度和最终选择结果，例如：

```text
Planning candidate 1/3 succeeded: joint-space length=2.1878 rad
Planning candidate 2/3 succeeded: joint-space length=4.8032 rad
Planning candidate 3/3 succeeded: joint-space length=12.5319 rad
Selected shortest of 3 valid candidates: joint-space length=2.1878 rad
```

规划失败或未通过碰撞验证的候选不会参与比较，更不会发送给控制器。如果 3 条候选
全部失败，本次运动会安全中止。候选数量由 `moveit_client.py` 中的
`PLANNING_CANDIDATES` 控制。

## Python 控制接口

`UR7eMoveItClient` 用于在同一个进程中连续执行多个动作。它只创建一次
MoveItPy，并为每一步自动完成当前状态获取、规划、执行及结果检查。

```python
import time

from ur7e_motion import UR7eMotionError, UR7eMoveItClient


READY = [
    1.54,
    -1.62,
    1.4,
    -1.350796327,
    -1.570796326589793,
    -0.030796327,
]
POINT_A_POSITION = [0.30, -0.20, 0.40]
POINT_B_POSITION = [0.30, 0.20, 0.40]
TOOL_ORIENTATION = [0.0, 1.0, 0.0, 0.0]  # 四元数 [x, y, z, w]

arm = UR7eMoveItClient(node)
try:
    arm.move_to_joint(READY)
    arm.move_to_pose(POINT_A_POSITION, TOOL_ORIENTATION)
    time.sleep(1.0)
    arm.move_to_pose(POINT_B_POSITION, TOOL_ORIENTATION)
    arm.move_by_translation([0.0, 0.0, 0.05])
    arm.move_to_joint(READY)
except UR7eMotionError as exc:
    node.get_logger().error(f"任务中止：{exc}")
```

接口含义：

- `move_to_joint()`：运动到六个绝对关节角，单位为弧度。
- `move_to_pose()`：运动到指定坐标系下的绝对末端位姿；位置单位为米，四元数
  顺序为 `[x, y, z, w]`。
- `move_by_translation()`：在 `base_link` 下相对当前 `robotiq_tcp` 位姿平移，并保持
  当前末端方向。

这些接口需要在已经初始化 `rclpy`、持有 ROS 节点并加载本包 MoveIt 参数的任务
节点中使用。规划或执行失败时会抛出 `UR7eMotionError` 的相应子类，任务代码应当
停止后续动作并报告错误。

## 夹爪 Python 控制接口

`GripperClient` 通过
`/gripper_controller/follow_joint_trajectory` 控制 Robotiq 2F-85。接口会同步等待
动作完成，因此可以直接按照任务顺序书写代码：

```python
import rclpy

from ur7e_motion import GripperClient, GripperControlError


rclpy.init()
node = rclpy.create_node("gripper_example")
gripper = GripperClient(node)
try:
    gripper.open(duration=2.0)
    gripper.move_to(0.4, duration=1.0)
    gripper.close(duration=2.0)
except GripperControlError as exc:
    node.get_logger().error(f"夹爪任务中止：{exc}")
finally:
    gripper.destroy()
    node.destroy_node()
    rclpy.shutdown()
```

接口含义：

- `open(duration)`：运动到 `0.0 rad`，即完全张开；
- `close(duration)`：运动到 `0.8 rad`，即完全闭合；
- `move_to(position, duration)`：运动到 `[0.0, 0.8] rad` 范围内的中间位置；
- `duration`：期望动作时间，单位为秒，必须大于零。

控制器不可用、目标被拒绝、执行超时或执行失败时会抛出
`GripperControlError` 的具体子类。调用方应停止后续抓取动作，而不是假定夹爪已经
到位。

## 固定任务示例

本包提供可直接运行的固定任务节点 `fixed_task`，动作顺序为：

1. 运动到 `READY` 关节位置。
2. `robotiq_tcp` 沿 `base_link` 的 Z 正方向移动 5 cm。
3. 沿 X 正方向移动 5 cm。
4. 沿 Y 正方向移动 5 cm。
5. 沿 X 负方向移动 5 cm。
6. 沿 Y 负方向移动 5 cm，完成一个方形的四个目标点。
7. 沿 Z 负方向移动 5 cm。
8. 再次运动到 `READY`。

每次相对移动后默认停留 1 秒。这里使用 OMPL 在相邻目标点之间规划无碰路径，
因此末端会依次到达方形顶点，但点与点之间不保证是严格的笛卡尔直线。

确保 Mock Hardware 和 MoveIt 已分别启动后，在工作区根目录的新终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_motion fixed_task.launch.py
```

可以修改每一步的停留时间，例如：

```bash
ros2 launch ur7e_motion fixed_task.launch.py pause_seconds:=2.0
```

固定点位和动作顺序位于 `ur7e_motion/fixed_task.py` 的常量和 `execute_task()` 中。
任务全过程只创建一个 `MoveItPy` 实例；任何一步状态获取、规划或执行失败，后续
动作都不会继续发送。

## 桌面与自动避障示例

`obstacle_demo` 会执行以下流程：

1. 运动到 `READY`。
2. 在 `base_link` 下加入一个位于安装平面下方的桌面碰撞体。
3. 根据当前 `robotiq_tcp` 位置，在末端与目标之间动态加入一个障碍物。
4. 保持当前末端姿态，规划到障碍物另一侧；OMPL 必须寻找无碰路径。
5. 在障碍物仍存在时返回 `READY`。
6. 删除本次演示加入的场景物体。

先按前文方式启动 Mock Hardware、MoveIt 和 RViz，然后在新终端运行：

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_motion obstacle_demo.launch.py
```

场景参数保存在 `config/obstacle_demo.yaml`：

- `table_size`、`table_position`：桌面尺寸和中心位置。
- `obstacle_size`：路径中障碍物的尺寸。
- `obstacle_offset`：障碍物相对演示起点 `robotiq_tcp` 的位置。
- `target_translation`：目标相对起点的平移量。
- `scene_wait_seconds`、`goal_hold_seconds`：场景同步和目标停留时间。

`SceneManager` 目前提供：

```python
scene.add_box("box", [0.1, 0.2, 0.3], [0.4, 0.0, 0.2])
scene.set_object_touch_allowed("box", touch_links, True)
scene.attach_object(
    "box", link_name="robotiq_tcp", touch_links=touch_links
)
scene.detach_object("box")
scene.set_object_touch_allowed("box", touch_links, False)
scene.remove_object("box")
scene.clear()
```

碰撞物体会写入本节点使用的 MoveIt 规划场景，同时通过 `/planning_scene` 发布差分。
如果规划失败，不会发送轨迹，演示加入的场景物体仍会在退出前清理。

## RViz 抓取与放置示例

`pick_place_demo` 使用 MoveIt 的 Attached Collision Object 模拟抓取。它不会计算
接触力或重力，而是在夹爪闭合后将方块逻辑附加到 `robotiq_tcp`，使方块跟随机械臂
运动并参与碰撞检测。

动作顺序为：

1. 到达夹爪严格向下的 `READY` 并张开夹爪；
2. 在当前 TCP 下方生成一个 35 mm 宽的方块；
3. 向下接近并将夹爪闭合到 `0.50 rad`；
4. 附加方块，抬升 10 cm 并沿 X 搬运 15 cm；
5. 下降、张开夹爪并解除附加；
6. 退出抓取区域并返回 `READY`。

启动 Mock Hardware 和 MoveIt/RViz 后，在第三个终端执行：

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_motion pick_place_demo.launch.py
```

参数位于 `config/pick_place_demo.yaml`。成功后方块保留在放置位置；再次运行会用同一
物体 ID 覆盖旧方块。规划、执行、夹爪或场景操作失败时，程序会停止后续动作并清理
本次临时附加状态和碰撞许可。

当前未配置 3D 相机，因此 MoveIt 启动时可能打印
`No 3D sensor plugin(s) defined for octomap updates`。本示例使用显式几何碰撞体，
不依赖 OctoMap，该提示不影响抓取演示。

## 统一工作单元抓取示例

`workspace_pick_place_demo` 会在 `base_link` 坐标系下建立一个固定工作单元：

- 位于机械臂前方的工作台；
- 工作台后方的墙壁；
- 左后侧储物箱；
- 阻挡 READY 到预抓取点直达路线的中间隔断；
- 台面上的抓取方块和另一处放置位置。

为方便观察，工作台显示为棕色、墙壁为灰蓝色、储物箱为蓝色、中间隔断为橙色、
抓取方块为红色。颜色只影响 RViz 显示，不会改变 MoveIt 的碰撞规则或规划结果。

它使用 MoveIt 的碰撞检测规划从严格向下的 `READY` 绕过中间隔断，到方块上方后
下移抓取、抬升、移动、放置并返回 READY。方块接触只对白名单中的 8 个 Robotiq
夹爪链路开放；桌面、墙壁、箱子、隔断及机械臂其他链路仍保持碰撞检测。

先按本文开头分别启动 Mock Hardware、MoveIt 和 RViz；然后在第三个终端执行：

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_motion workspace_pick_place_demo.launch.py
```

场景与任务参数位于 `config/workspace_pick_place_demo.yaml`。成功后桌面、墙壁、箱子、
隔断和放置后的方块都会保留在 MoveIt PlanningScene 中，便于在 RViz 查看；再次运行
会用相同 ID 更新它们。若规划、执行、夹爪或场景操作失败，程序立即停止后续动作，
并只清除本次目标方块和临时接触许可，静态工作单元仍保留。

这里的“抓取”是 MoveIt Attached Collision Object 的逻辑附加：方块会跟随
`robotiq_tcp` 并参与碰撞检测，但不会模拟重力、摩擦、真实夹持力或刚体物理。
