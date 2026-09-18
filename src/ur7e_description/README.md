# ur7e_description

这个包组合官方 `ur_description` 与 `robotiq_description`，描述安装在 UR7e
`flange` 上的 Robotiq 2F-85 两指夹爪。

模型包含：

- UR7e 的视觉、碰撞和惯性模型；
- UR 到 Robotiq 的机械转接板视觉与碰撞网格；
- Robotiq 2F-85 本体、连杆、指尖的视觉与碰撞网格；
- 主动关节 `robotiq_85_left_knuckle_joint`；
- 由主动关节驱动的 5 个 mimic 关节；
- 左右手指中间的固定关节。
- 位于两指中心、沿夹爪纵轴的任务坐标系 `robotiq_tcp`；
- UR7e 与夹爪各自的 `ros2_control` Mock Hardware 接口；
- 机械臂轨迹控制器和带时间插值的夹爪轨迹控制器。

## 安装依赖

```bash
sudo apt-get install \
  ros-jazzy-robotiq-description \
  ros-jazzy-joint-trajectory-controller
```

## 构建

```bash
cd /home/wenqin/project/ur_control
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select ur7e_description ur7e_moveit_config ur7e_motion
source install/setup.bash
```

## 在 RViz 中检查模型

```bash
ros2 launch ur7e_description view_ur7e_robotiq.launch.py
```

启动后可以使用 Joint State Publisher GUI 调整
`robotiq_85_left_knuckle_joint`，其余活动手指关节会按照 mimic 关系同步运动。

夹爪仍然安装在 `flange`，但使用 UR 官方 `flange → tool0` 的固定旋转
`roll=π/2, pitch=0, yaw=π/2`，让夹爪纵轴与 `tool0` 指向一致。如需绕已经对齐的
工具轴旋转夹爪正面，可以覆盖 `gripper_rotation`；也可以通过
`gripper_mount_roll`、`gripper_mount_pitch` 和 `gripper_mount_yaw` 调整安装变换。

## 启动 Mock Hardware 和控制器

展示模型的 launch 不能执行轨迹。需要控制机械臂和夹爪时使用：

```bash
ros2 launch ur7e_description ur7e_mock_control.launch.py
```

它会启动：

- `joint_state_broadcaster`；
- `scaled_joint_trajectory_controller`；
- `gripper_controller`。

可以在另一个终端确认：

```bash
ros2 control list_controllers
```

测试夹爪闭合：

```bash
ros2 action send_goal \
  /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [robotiq_85_left_knuckle_joint], points: [{positions: [0.8], time_from_start: {sec: 2}}]}}"
```

测试夹爪张开：

```bash
ros2 action send_goal \
  /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [robotiq_85_left_knuckle_joint], points: [{positions: [0.0], time_from_start: {sec: 2}}]}}"
```

这里的 `position` 是左侧主 knuckle 关节的弧度值，不是两指间距；`sec: 2`
表示用约 2 秒完成开合动画。当前阶段使用 Mock Hardware，不模拟抓取力、接触或
物体动力学。真机阶段会将这个轨迹控制器替换为 Robotiq 的夹爪动作控制器。
