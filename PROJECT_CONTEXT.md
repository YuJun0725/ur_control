# UR7e ROS 2 项目上下文

## 项目目标

建立通用机械臂控制系统：先完成仿真，再部署真机。上层接口尽量统一，支持运动控制、轨迹规划、避障、末端夹爪和摄像头，后续可扩展具身智能研究。

## 当前环境

- Ubuntu 24.04.3 LTS
- ROS 2 Jazzy
- UR7e
- ROS 官方 UR 二进制包通过 apt 安装
- 系统 ROS 环境：`/opt/ros/jazzy`
- 当前使用 `ros2_control` Mock Hardware，不连接 URSim 或真实机械臂

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

## 当前 ROS 2 链路

```text
MoveIt 2 → ros2_control 控制器 → Mock Hardware → /joint_states
                                      ↓
                             robot_state_publisher → /tf
```

真实 UR 链路将把 Mock Hardware 替换为 `ur_robot_driver` 的真实硬件接口。

## 下一步

1. 加入摄像头坐标系和物体坐标变换，用感知结果替代固定抓取点。
2. 为抓取接近和退出增加笛卡尔直线轨迹，并加入抓取前后安全点。
3. 再考虑 Gazebo/Isaac Sim 物理仿真；URSim 主要用于 UR 控制器和通信验证。
4. 创建 `ur7e_bringup` 包，整理多节点启动文件和工作单元配置。
5. 最后配置真实机器人网络、External Control、标定和真机驱动。

## 注意事项

- 不要修改 `/opt/ros/jazzy` 中的官方安装文件；自己的代码放在工作区 `src/`。
- `robot_ip:=0.0.0.0` 只用于 Mock Hardware 占位；真机必须使用控制柜实际 IP。
- 不要同时运行直接 Python URScript 控制程序和 ROS 2 驱动。
- 直接 Python 控制路线使用 Dashboard `29999`、RTDE `30004`、Secondary/URScript `30002`；目前项目主路线是 ROS 2 官方驱动。
