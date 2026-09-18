# UR7e ROS 2 项目上下文

## 项目目标

建立通用机械臂控制系统：先完成仿真，再部署真机。上层接口尽量统一，支持运动控制、轨迹规划、避障、末端夹爪和摄像头，后续可扩展具身智能研究。

## 当前环境

- Ubuntu 24.04.3 LTS
- ROS 2 Jazzy
- UR7e
- ROS 官方 UR 二进制包通过 apt 安装
- 系统 ROS 环境：`/opt/ros/jazzy`
- 当前同时保留 `ros2_control` Mock Hardware 流程，并已加入 Gazebo Harmonic
  物理仿真；尚未连接 URSim 或真实机械臂

## 已验证内容

1. `ur_description`、`ur_robot_driver`、`ur_moveit_config` 可以正常找到。
2. 曾遇到 Fast-CDR ABI 错误，升级 Fast-CDR/FastRTPS 相关 ROS 包后已解决。
3. 已成功启动：

   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 launch ur_robot_driver ur_control.launch.py \
     ur_type:=ur7e \
     robot_ip:=0.0.0.0 \
     use_mock_hardware:=true
   ```
4. `joint_state_broadcaster` 和 `scaled_joint_trajectory_controller` 已正常运行。
5. `/joint_states` 可以发布关节名称和状态。
6. MoveIt 2 和 RViz 已成功启动，可以设置目标姿态、规划并执行轨迹，RViz 中的模拟机械臂会运动。
7. 已建立 `ur7e_motion`，支持关节目标、末端相对运动、固定任务、Planning Scene
   障碍物和自动绕障示例。
8. 已建立 `ur7e_description`，组合官方 UR7e 与 Robotiq 2F-85 描述。夹爪通过
   官方机械转接板安装到 `flange`，视觉/碰撞网格、主动 knuckle 关节和 mimic
   关节均已加入并通过 URDF 检查。
9. UR7e 和夹爪均已接入 `ros2_control` Mock Hardware；
   `scaled_joint_trajectory_controller`、`gripper_controller` 和
   `joint_state_broadcaster` 已通过实际启动验证。
10. 已建立 `ur7e_moveit_config`，加入 `gripper` 规划组、Robotiq 末端执行器及
    `robotiq_tcp`。机械臂 `ur_manipulator` 的规划链现为
    `base_link → robotiq_tcp`，关节轨迹规划与执行已经验证成功。
11. 已封装 `GripperClient`，支持同步 `open()`、`close()` 和 `move_to()`，并具有
    Action 服务发现、拒绝、超时和执行失败处理。
12. 已实现 RViz 逻辑抓取与放置演示：世界方块可附加到 `robotiq_tcp`、随机器人
    搬运并在放置点解除附加；临时触碰碰撞许可会在退出前恢复。
13. 任务 READY 已统一为夹爪严格向下的姿态，`robotiq_tcp` 工具 Z 轴经 TF 验证为
    `[0, 0, -1]`。
14. 已实现 `workspace_pick_place_demo`：在 MoveIt PlanningScene 中建立前方工作台、
    后墙、储物箱、中间隔断和抓取方块。实际 Mock Hardware 验证中，机械臂成功绕开
    隔断、抓取并搬运方块、无碰撞抬升、放置后解除附加并返回严格向下 READY；静态场景
    和放置方块会保留在 RViz。
15. 已建立 `ur7e_gazebo`，接入 Gazebo Harmonic、`gz_ros2_control`、仿真时钟和
    工作单元物理场景。UR7e 六轴和 Robotiq 主关节由 Gazebo 控制，同时保留现有
    MoveIt 控制器名称与 Python 接口。
16. Gazebo 完整抓取任务已通过无界面实测：三个控制器均进入 `active`，机械臂绕开
    隔断完成抓取、抬升、搬运、放置并返回 READY；80 g 动态方块从约
    `[0.16, 0.47, 0.33]` 被物理搬运到 `[0.1600, 0.3213, 0.3300] m`。
17. Gazebo 物理引擎使用 Bullet Featherstone、1 ms 步长和实时倍率 1.0。未使用
    原计划中的 DART，因为 Harmonic 的 DART 后端不能创建夹爪所需的 mimic 约束；
    Bullet Featherstone 能正确驱动五个被动夹爪关节。

## 当前 ROS 2 链路

```text
                            ┌→ Mock Hardware
MoveIt 2 → ros2_control ──┤
                            └→ gz_ros2_control → Gazebo 物理世界
                                     ↓
                               /joint_states
                                     ↓
                          robot_state_publisher → /tf
```

Mock Hardware 与 Gazebo 互斥启动。真实 UR 链路将把仿真硬件替换为
`ur_robot_driver` 的真实硬件接口。

## 下一步

1. 在 Gazebo 中加入 RGB-D 相机模型，并桥接图像、深度图和 CameraInfo。
2. 加入相机坐标系、目标检测和 TF 坐标变换，用感知结果替代固定抓取点。
3. 为抓取接近和退出增加笛卡尔直线轨迹，并加入抓取前后安全点。
4. 调整夹爪接触、控制器跟踪和异常恢复，提高不同初始状态下的抓取稳定性。
5. 最后配置真实机器人网络、External Control、标定和真机驱动。

## 注意事项

- 不要修改 `/opt/ros/jazzy` 中的官方安装文件；自己的代码放在工作区 `src/`。
- `robot_ip:=0.0.0.0` 只用于 Mock Hardware 占位；真机必须使用控制柜实际 IP。
- 不要同时运行直接 Python URScript 控制程序和 ROS 2 驱动。
- 不要同时启动 Mock Hardware 和 Gazebo；两者使用相同的控制器、关节状态和 TF 名称。
- 直接 Python 控制路线使用 Dashboard `29999`、RTDE `30004`、Secondary/URScript `30002`；目前项目主路线是 ROS 2 官方驱动。
