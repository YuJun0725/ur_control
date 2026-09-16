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

终端 1：启动 UR7e Mock Hardware 和 `ros2_control` 控制器。

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e robot_ip:=0.0.0.0 use_mock_hardware:=true launch_rviz:=false
```

终端 2：启动 MoveIt 2 和 RViz。

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur7e launch_rviz:=true
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
  joint_positions:="[1.54, -1.62, 1.4, -1.2, -1.6, -0.11]"
```

### 末端位姿偏移

以下命令在保持当前末端姿态的同时，使 `tool0` 沿 `base_link` 坐标系的 Z
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

## Python 控制接口

`UR7eMoveItClient` 用于在同一个进程中连续执行多个动作。它只创建一次
MoveItPy，并为每一步自动完成当前状态获取、规划、执行及结果检查。

```python
import time

from ur7e_motion import UR7eMotionError, UR7eMoveItClient


READY = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
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
- `move_by_translation()`：在 `base_link` 下相对当前 `tool0` 位姿平移，并保持
  当前末端方向。

这些接口需要在已经初始化 `rclpy`、持有 ROS 节点并加载本包 MoveIt 参数的任务
节点中使用。规划或执行失败时会抛出 `UR7eMotionError` 的相应子类，任务代码应当
停止后续动作并报告错误。

## 固定任务示例

本包提供可直接运行的固定任务节点 `fixed_task`，动作顺序为：

1. 运动到 `READY` 关节位置。
2. `tool0` 沿 `base_link` 的 Z 正方向移动 5 cm。
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
