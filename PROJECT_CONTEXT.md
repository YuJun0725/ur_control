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

## 当前 ROS 2 链路

```text
MoveIt 2 → ros2_control 控制器 → Mock Hardware → /joint_states
                                      ↓
                             robot_state_publisher → /tf
```

真实 UR 链路将把 Mock Hardware 替换为 `ur_robot_driver` 的真实硬件接口。

## 下一步等

1. 建立自己的工作区：`~/ur_ws/src/`。
2. 创建 `ur7e_bringup` 包，保存启动文件和配置。
3. 创建 `ur7e_motion` 包，用程序调用 MoveIt 规划和执行，而不是只用 RViz 鼠标操作。
4. 加入 Planning Scene：桌面、方块、货架等障碍物，并测试避障。
5. 加入末端工具和夹爪模型，设计统一的 `open()`、`close()` 等接口。
6. 加入摄像头坐标系和物体坐标变换。
7. 再考虑 Gazebo/Isaac Sim 物理仿真；URSim 主要用于 UR 控制器和通信验证。
8. 最后配置真实机器人网络、External Control、标定和真机驱动。

## 注意事项

- 不要修改 `/opt/ros/jazzy` 中的官方安装文件；自己的代码放在工作区 `src/`。
- `robot_ip:=0.0.0.0` 只用于 Mock Hardware 占位；真机必须使用控制柜实际 IP。
- 不要同时运行直接 Python URScript 控制程序和 ROS 2 驱动。
- 直接 Python 控制路线使用 Dashboard `29999`、RTDE `30004`、Secondary/URScript `30002`；目前项目主路线是 ROS 2 官方驱动。
