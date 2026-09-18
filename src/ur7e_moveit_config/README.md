# UR7e + Robotiq 2F-85 MoveIt 配置

这个包为组合机器人提供：

- `ur_manipulator`：从 `base_link` 到 `robotiq_tcp` 的机械臂规划组；
- `gripper`：由 `robotiq_85_left_knuckle_joint` 驱动的夹爪规划组；
- `robotiq_2f_85`：挂接在 `flange` 上的 MoveIt 末端执行器；
- `scaled_joint_trajectory_controller` 与 `gripper_controller` 的执行映射；
- OMPL 和 KDL 运动学配置。

先启动 Mock Hardware：

```bash
ros2 launch ur7e_description ur7e_mock_control.launch.py
```

再启动 MoveIt：

```bash
ros2 launch ur7e_moveit_config ur7e_moveit.launch.py
```
