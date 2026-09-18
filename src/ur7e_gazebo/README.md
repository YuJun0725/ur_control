# ur7e_gazebo

这个包将项目现有的 UR7e、Robotiq 2F-85、MoveIt 和 Python 任务接入
Gazebo Harmonic。Gazebo 负责刚体动力学、接触和摩擦；MoveIt 继续负责运动规划、
碰撞检查以及抓取期间的逻辑附加物体。

## 仿真内容

- UR7e 六个关节和 Robotiq 主 knuckle 关节由 `gz_ros2_control` 驱动；
- 保留 `/scaled_joint_trajectory_controller/follow_joint_trajectory` 和
  `/gripper_controller/follow_joint_trajectory`，现有 MoveIt 与 Python 接口不变；
- 世界中包含桌面、后墙、储物箱、中间隔断和一个 80 g 的动态方块；
- 物理引擎使用 Bullet Featherstone；Gazebo Harmonic 的 DART 尚不支持夹爪所需的
  mimic 约束，而 Bullet Featherstone 可以让五个被动夹爪关节跟随主动关节；
- 方块依靠夹爪碰撞网格与摩擦被夹起，不使用固定关节把方块粘在夹爪上；
- 首版桥接 `/clock` 和重复任务所需的 `/world/default/set_pose` 服务，尚未加入
  相机和图像话题。

不要同时启动 `ur7e_mock_control.launch.py` 和 Gazebo，两者会争用相同的控制器、
`/joint_states` 和机器人 TF。

## 构建

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur7e_gazebo
source install/setup.bash
```

## 一键运行

先关闭以前启动的 Mock Hardware、MoveIt 和 RViz，然后执行：

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_gazebo ur7e_gazebo_demo.launch.py
```

启动器会等待三个控制器全部进入 `active`，再启动 MoveIt、RViz，最后运行一次
抓取放置任务。常用选项：

```bash
# 只启动完整仿真与 MoveIt，不自动执行任务
ros2 launch ur7e_gazebo ur7e_gazebo_demo.launch.py run_demo:=false

# 无 Gazebo GUI、无 RViz，适合检查启动或降低笔记本负载
ros2 launch ur7e_gazebo ur7e_gazebo_demo.launch.py \
  gui:=false launch_rviz:=false run_demo:=false
```

`paused:=true` 会让 Gazebo 停在暂停状态。执行运动任务前需要在 Gazebo 中点击
播放，否则依赖仿真时间的状态和轨迹不会推进。

## 三终端模块化运行

终端 1：启动 Gazebo、机器人、时钟桥和控制器。

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_gazebo ur7e_gazebo.launch.py
```

终端 2：等待终端 1 中三个控制器加载完成，再启动 MoveIt 和 RViz。

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_moveit_config ur7e_moveit.launch.py use_sim_time:=true
```

终端 3：运行一次物理抓取任务。每次启动都会先把 Gazebo 方块重置到抓取位置。

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur7e_gazebo workspace_pick_place_gazebo.launch.py
```

可以用下面的命令确认控制器状态：

```bash
ros2 control list_controllers
```

预期三个控制器均为 `active`：

- `joint_state_broadcaster`
- `scaled_joint_trajectory_controller`
- `gripper_controller`

## 场景坐标

所有坐标均位于 `base_link`，单位为米：

| 物体 | 中心位置 | 尺寸 |
|---|---:|---:|
| 桌面 | `[0.00, 0.48, 0.15]` | `[0.70, 0.45, 0.30]` |
| 后墙 | `[0.00, 0.75, 0.575]` | `[0.70, 0.05, 0.55]` |
| 储物箱 | `[-0.26, 0.62, 0.40]` | `[0.14, 0.14, 0.20]` |
| 中间隔断 | `[0.02, 0.47, 0.42]` | `[0.06, 0.18, 0.24]` |
| 抓取方块 | `[0.16, 0.47, 0.331]` | `[0.035, 0.035, 0.060]` |

方块被放到 `[0.16, 0.32, 0.331]` 附近。Gazebo 中它会受重力和接触影响，
因此实际稳定位置可能与规划坐标存在毫米级差异。

## 当前边界

MoveIt 的“附加物体”只影响规划场景，不会直接移动 Gazebo 方块。Gazebo 中能否夹起
方块由实际碰撞、摩擦、夹爪闭合角度和控制器跟踪共同决定。如果方块滑落，应优先
调节 `workspace_pick_place_gazebo.yaml` 中的抓取位置与时长，或 Xacro/世界中的摩擦
参数，而不是添加固定关节。当前 35 mm 方块使用 `0.37 rad`，比 RViz 逻辑演示的
`0.50 rad` 更小，以免控制器要求手指穿入实体方块。

Gazebo 的位置接口存在一阶跟踪滞后，因此 Gazebo 专用任务把 MoveIt 的轨迹起点容差
设为 `0.1 rad`，与机械臂控制器的最终目标容差一致；Mock Hardware 与未来真机仍保留
原来的 `0.01 rad`。
