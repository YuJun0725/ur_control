# UR7e 控制项目文件与架构说明

本文档用于回答三个问题：

1. 工作区中的每个主要文件负责什么；
2. 文件之间如何组合成一个可运行的 ROS 2 系统；
3. 一条 Python 运动命令最终如何让 Gazebo 或 Mock Hardware 中的机械臂运动。

项目当前使用 ROS 2 Jazzy、MoveIt 2、ros2_control、Gazebo Harmonic、UR7e 和
Robotiq 2F-85。系统同时保留 Mock Hardware 和 Gazebo 两种运行方式，但二者不能同时
启动。

## 1. 先建立整体认识

整个项目可以分成五层：

```text
感知层
ur7e_vision：RGB-D 图像、颜色分割、深度反投影和 TF 坐标转换
        ↓ 输出 base_link 下的目标坐标

任务层
ur7e_motion 中的 motion_command / fixed_task / pick-place 节点
        │
        ├── UR7eMoveItClient：机械臂规划与执行
        ├── GripperClient：夹爪轨迹 Action
        └── SceneManager：MoveIt 逻辑碰撞场景
        │
        ▼
规划层
MoveItPy / move_group / OMPL / PlanningScene
        │
        ▼
控制层
ros2_control
├── scaled_joint_trajectory_controller
└── gripper_controller
        │
        ▼
硬件或仿真层
├── Mock Hardware：关节状态直接跟随命令，不计算物理
└── gz_ros2_control：驱动 Gazebo 中的关节、碰撞、接触和摩擦
```

五个 ROS 2 包的职责如下：

| 包                     | 职责                   | 回答的问题                                 |
| ---------------------- | ---------------------- | ------------------------------------------ |
| `ur7e_description`   | 机器人模型与 Mock 控制 | 机器人由哪些 link/joint 构成，夹爪装在哪里 |
| `ur7e_moveit_config` | MoveIt 语义与规划配置  | 哪些关节一起规划，TCP 是谁，轨迹发给谁     |
| `ur7e_motion`        | Python 控制接口和任务  | 如何用代码完成运动、避障、抓取和放置       |
| `ur7e_gazebo`        | Gazebo 物理仿真        | 物理世界、控制器和动态方块怎样启动         |
| `ur7e_vision`        | RGB-D 视觉感知         | 如何识别彩色方块并求出其三维坐标           |

## 2. 工作区根目录

```text
ur_control/
├── ARCHITECTURE.md      # 本文档：代码和运行架构地图
├── .gitignore           # 排除构建产物、缓存、日志和本机配置
├── src/                 # 所有需要纳入 Git 的源代码
├── build/               # colcon 编译中间产物，不提交 Git
├── install/             # 构建后的可运行安装空间，不提交 Git
└── log/                 # colcon 构建和测试日志，不提交 Git
```

运行 `colcon build --symlink-install` 后，ROS 2 实际通过 `install/` 查找包。每次打开新
终端都需要执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenqin/project/ur_control/install/setup.bash
```

`source` 不会编译代码，只是把系统 ROS 和当前工作区加入终端环境。

## 3. `ur7e_description`：机器人长什么样

目录：`src/ur7e_description/`

### 3.1 包级文件

| 文件               | 作用                                                                             |
| ------------------ | -------------------------------------------------------------------------------- |
| `package.xml`    | 声明包名、版本、许可证以及 UR、Robotiq、ros2_control 等运行依赖                  |
| `CMakeLists.txt` | 告诉`ament_cmake` 将 `urdf/`、`config/`、`launch/` 安装到包的 share 目录 |
| `README.md`      | 模型查看、Mock Hardware、夹爪测试和 Gazebo 分支的使用说明                        |

### 3.2 机器人模型

#### `urdf/ur7e_robotiq_2f_85.urdf.xacro`

这是整个项目的机器人模型入口，也是最底层的核心文件。它负责：

- 调用官方 `ur_description` 宏生成 UR7e；
- 调用官方 `robotiq_description` 宏生成 Robotiq 2F-85；
- 将夹爪通过转接板安装到 UR7e 的 `flange`；
- 修正夹爪方向，使其纵轴与 UR 的 `tool0` 对齐；
- 建立两指中心任务坐标系 `robotiq_tcp`；
- 保留夹爪主动关节和五个 mimic 关节关系；
- 在 Mock 模式下声明 UR 和夹爪的 GenericSystem；
- 在 Gazebo 模式下声明统一的 `GazeboSimSystem`；
- 为 Gazebo 指尖设置摩擦系数。

同一个 Xacro 根据参数生成不同控制接口：

```text
use_mock_hardware=true, use_gazebo=false
    → Mock Hardware

use_mock_hardware=false, use_gazebo=true
    → gz_ros2_control / GazeboSimSystem

use_mock_hardware=false, use_gazebo=false
    → 为未来真实 UR 驱动保留
```

Xacro 只描述机器人，不会自己启动任何节点。它需要被 launch 文件执行并展开成
`robot_description`。

### 3.3 Mock 控制配置

#### `config/mock_controllers.yaml`

供 Mock Hardware 使用，定义：

- `joint_state_broadcaster`：发布 `/joint_states`；
- `scaled_joint_trajectory_controller`：接收 UR7e 六关节轨迹；
- `gripper_controller`：接收夹爪主动关节轨迹；
- 控制器关节列表、命令接口、状态接口和更新频率。

该文件真正创建的是 ros2_control 控制器；不要与
`ur7e_moveit_config/config/moveit_controllers.yaml` 混淆，后者只是告诉 MoveIt 去哪里
寻找这些控制器。

### 3.4 Launch 文件

#### `launch/view_ur7e_robotiq.launch.py`

只用于查看模型：

- 展开 Xacro；
- 启动 `robot_state_publisher`；
- 启动 Joint State Publisher GUI；
- 启动 RViz。

它适合检查夹爪安装方向、TF 和 mimic 开合，不具备轨迹执行能力。

#### `launch/ur7e_mock_control.launch.py`

启动 Mock 控制链：

1. 展开组合 Xacro；
2. 启动 `robot_state_publisher`；
3. 启动独立的 `ros2_control_node`；
4. 加载并激活三个控制器。

它不启动 MoveIt、RViz 或任务节点。

## 4. `ur7e_moveit_config`：MoveIt 如何理解机器人

目录：`src/ur7e_moveit_config/`

### 4.1 包级文件

| 文件                 | 作用                                                |
| -------------------- | --------------------------------------------------- |
| `package.xml`      | 声明 MoveIt、RViz、机器人描述包等依赖               |
| `CMakeLists.txt`   | 安装`config/`、`launch/`、`srdf/` 等配置资源  |
| `.setup_assistant` | MoveIt Setup Assistant 生成的包元数据，记录配置来源 |
| `README.md`        | MoveIt 配置和启动方式说明                           |

### 4.2 SRDF 语义模型

#### `srdf/ur7e_robotiq.srdf`

URDF 说明“机器人有什么”，SRDF 说明“MoveIt 应该怎样使用它”。该文件定义：

- `ur_manipulator` 机械臂规划组；
- `gripper` 夹爪规划组；
- Robotiq 夹爪末端执行器，以及作为机械臂规划链末端的 `robotiq_tcp`；
- 严格向下的 READY 命名关节状态；
- 不需要做碰撞检测的相邻 link 对。

其中机械臂规划链为：

```text
base_link → ... → flange → Robotiq 夹爪 → robotiq_tcp
```

因此 `move_to_pose()` 控制的是夹爪中心，而不是裸露的 `tool0`。

### 4.3 MoveIt 配置文件

#### `config/joint_limits.yaml`

补充或覆盖 MoveIt 使用的速度、加速度限制。当前为 UR7e 六关节设置最大加速度，并为
夹爪主动关节设置速度和加速度限制。它影响轨迹时间参数化，不是 Gazebo PID，也不是
真机安全限制。

#### `config/kinematics.yaml`

为 `ur_manipulator` 指定 KDL 逆运动学求解器。MoveIt 用它把
`robotiq_tcp` 的目标位置和姿态转换成六个机械臂关节角。

#### `config/moveit_controllers.yaml`

建立 MoveIt 与现有控制器之间的映射：

```text
UR7e 轨迹
  → /scaled_joint_trajectory_controller/follow_joint_trajectory

夹爪轨迹
  → /gripper_controller/follow_joint_trajectory
```

它还设置轨迹执行时间余量和起点容差，但不会创建控制器。

#### `config/ompl_planning.yaml`

加载 OMPL 规划插件，以及规划前后的适配器：

```text
解析约束坐标系
→ 检查工作空间
→ 检查起点关节限位
→ 检查起点碰撞
→ OMPL 搜索
→ 添加轨迹时间
→ 验证结果
→ 发布显示轨迹
```

### 4.4 MoveIt 启动文件

#### `launch/ur7e_moveit.launch.py`

使用 `MoveItConfigsBuilder` 汇总 URDF、SRDF、运动学、关节限位、OMPL 和控制器配置，
然后启动：

- `move_group`：为 RViz MotionPlanning 面板提供规划、执行和场景接口；
- `rviz2`：显示机器人、规划轨迹和 PlanningScene。

参数：

- `launch_rviz`：是否启动 RViz；
- `use_sim_time`：是否使用 Gazebo `/clock`。

Python 任务内部的 MoveItPy 会创建自己的 MoveItCpp 实例。它并不是把每个规划请求发送
给 `/move_group`；MoveItPy 和 `move_group` 使用同一套模型、规划配置、场景话题和控制器
接口。当前仍启动 `move_group`，主要是为了 RViz MotionPlanning 面板和统一场景观察。

## 5. `ur7e_motion`：Python 控制和任务逻辑

目录：`src/ur7e_motion/`

这是上层业务逻辑包，也是以后增加新机器人任务时最常修改的包。

### 5.1 Python 包与安装文件

| 文件                        | 作用                                                   |
| --------------------------- | ------------------------------------------------------ |
| `package.xml`             | ROS 2 包依赖声明                                       |
| `setup.py`                | 声明 Python 包、安装 YAML/launch，并注册五个可执行程序 |
| `setup.cfg`               | 指定 ROS 2 Python 可执行文件的安装位置                 |
| `resource/ur7e_motion`    | ament 资源索引标记，使 ROS 2 能发现该包                |
| `ur7e_motion/__init__.py` | 对外导出常用客户端和异常类                             |
| `README.md`               | 构建、启动、接口和示例说明                             |

`setup.py` 不需要手动执行。`colcon build` 会读取它；修改 Python 文件后使用
`--symlink-install` 时通常无需重新复制源码，但新增入口、launch、YAML 或依赖后应重新
构建并重新 source。

`setup.py` 注册的可执行程序为：

```text
motion_command
fixed_task
obstacle_demo
pick_place_demo
workspace_pick_place_demo
```

### 5.2 核心 Python 模块

#### `ur7e_motion/validation.py`

公共输入验证工具：

- 检查向量长度；
- 检查是否为有限数值；
- 归一化四元数；
- 检查 `joint` / `pose` 模式。

它阻止 NaN、无穷值、错误维度等数据进入 MoveIt。

#### `ur7e_motion/moveit_client.py`

机械臂高层控制封装 `UR7eMoveItClient`：

- `move_to_joint()`：移动到六个绝对关节角；
- `move_to_pose()`：移动 `robotiq_tcp` 到绝对位姿；
- `move_by_translation()`：从当前 TCP 相对平移并保持姿态；
- `get_current_pose()`：读取当前 TCP 位姿；
- 每次运动前取得最新实际机器人状态；
- 每个目标使用 OMPL 规划三次；
- 选择累计关节空间长度最短的有效轨迹；
- 只在规划成功后调用 MoveIt 执行轨迹。

它依赖 launch 文件传入 MoveIt 参数，自身不会读取所有 YAML。

#### `ur7e_motion/gripper_client.py`

夹爪同步 Action 客户端 `GripperClient`：

- `open()`：移动到 `0.0 rad`；
- `close()`：移动到 `0.8 rad`；
- `move_to()`：移动到指定中间角度；
- 等待 Action 服务发现；
- 区分目标拒绝、发送超时、执行超时和控制器失败；
- 超时时取消旧目标。

它发送到：

```text
/gripper_controller/follow_joint_trajectory
```

#### `ur7e_motion/scene_manager.py`

MoveIt PlanningScene 封装 `SceneManager`：

- `add_box()`：添加桌面、墙、隔断或目标方块；
- `remove_object()`：移除物体；
- `attach_object()`：将世界物体逻辑附着到 `robotiq_tcp`；
- `detach_object()`：解除附着并保留当前世界位姿；
- `set_object_touch_allowed()`：修改 Allowed Collision Matrix；
- `clear()`：清理由本管理器创建的物体。

它同时更新 MoveItPy 的本地 PlanningSceneMonitor，并向 `/planning_scene` 发布增量，
使 `move_group` 和 RViz 同步。

### 5.3 任务程序

#### `ur7e_motion/motion_command.py`

最小单次运动入口。ROS 参数 `mode` 决定：

- `joint`：执行 `move_to_joint(joint_positions)`；
- `pose`：执行 `move_by_translation(translation)`。

执行一次后退出，并用不同退出码区分参数、状态、规划和执行失败。

#### `ur7e_motion/fixed_task.py`

固定动作序列示例：到 READY，依次沿 Z/X/Y 方向移动形成多个点，最后回 READY。它展示
如何只创建一个 `UR7eMoveItClient` 并连续调用高层运动函数。

READY 常量也定义在这里，其他抓取任务复用它，保证不同任务使用同一严格向下姿态。

#### `ur7e_motion/obstacle_demo.py`

基础避障示例：

- 回 READY；
- 添加桌面和动态障碍物；
- 规划到障碍物另一侧；
- 返回 READY；
- 清理演示物体。

用于验证 PlanningScene 和 OMPL 是否能生成无碰撞路径。

#### `ur7e_motion/pick_place_demo.py`

基础 RViz 逻辑抓取示例：

- 在 READY 下方生成目标方块；
- 允许方块与 8 个夹爪 link 接触；
- 接近、闭合、附着、抬升、搬运、放置；
- 解除附着并回 READY。

该文件同时定义 `GRIPPER_TOUCH_LINKS` 和 `TCP_LINK`，完整工作单元任务复用这些常量。

#### `ur7e_motion/workspace_pick_place_demo.py`

当前最完整的任务程序。它将三个核心类组合起来：

```text
UR7eMoveItClient + GripperClient + SceneManager
```

任务流程：

```text
READY
→ 张开夹爪
→ 建立桌面/墙/箱子/隔断/方块
→ 绕障到预抓取点
→ 下到抓取点
→ 闭合夹爪
→ 将方块逻辑附着到 TCP
→ 抬升
→ 搬运到放置区上方
→ 下放
→ 张开并解除附着
→ 退出
→ 清除临时触碰许可
→ READY
```

任意步骤失败都会停止后续动作；如果方块已经附着，会尽力解除附着、撤销触碰许可并
清理动态目标。成功时保留场景和放置后的方块。

### 5.4 任务配置文件

| 文件                                      | 使用者                 | 作用                                                                                                                         |
| ----------------------------------------- | ---------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `config/motion_planning.yaml`           | 所有 MoveItPy 任务     | PlanningSceneMonitor、RRTConnect、5 秒规划时间、速度/加速度缩放 0.2；每次`plan()` 内尝试一次，客户端在外层调用三次生成候选 |
| `config/obstacle_demo.yaml`             | `obstacle_demo`      | 桌面、障碍物尺寸、目标偏移等                                                                                                 |
| `config/pick_place_demo.yaml`           | `pick_place_demo`    | 方块尺寸、抓取/抬升/搬运参数和夹爪角度                                                                                       |
| `config/workspace_pick_place_demo.yaml` | Mock/RViz 工作单元任务 | 桌面、墙、箱子、隔断、抓取点和放置点                                                                                         |

### 5.5 任务 Launch 文件

| 文件                                           | 启动的可执行程序              | 额外工作                                        |
| ---------------------------------------------- | ----------------------------- | ----------------------------------------------- |
| `launch/motion_command.launch.py`            | `motion_command`            | 接收 mode、关节目标、平移参数并加载 MoveIt 配置 |
| `launch/fixed_task.launch.py`                | `fixed_task`                | 加载 MoveIt 参数和动作停留时间                  |
| `launch/obstacle_demo.launch.py`             | `obstacle_demo`             | 加载规划参数及障碍物 YAML                       |
| `launch/pick_place_demo.launch.py`           | `pick_place_demo`           | 加载规划参数及抓取 YAML                         |
| `launch/workspace_pick_place_demo.launch.py` | `workspace_pick_place_demo` | 加载规划参数及完整工作单元 YAML                 |

这些 launch 会把 MoveIt 参数传给任务进程中的 MoveItPy，但不负责启动 Mock Hardware、
Gazebo、外部 `move_group` 或 RViz。

### 5.6 单元测试

| 文件                                       | 覆盖内容                                   |
| ------------------------------------------ | ------------------------------------------ |
| `test/test_validation.py`                | 数值向量、模式和四元数验证                 |
| `test/test_moveit_client.py`             | 多候选轨迹长度和错误处理                   |
| `test/test_gripper_client.py`            | 夹爪 Action 成功、拒绝、超时和失败         |
| `test/test_scene_manager.py`             | 物体添加、附着、解除附着和碰撞许可         |
| `test/test_obstacle_demo.py`             | 避障任务调用顺序与失败中止                 |
| `test/test_pick_place_demo.py`           | 基础抓取流程和异常清理                     |
| `test/test_workspace_pick_place_demo.py` | 完整工作单元流程、重复场景及各阶段失败处理 |

测试使用假的 arm/gripper/scene 对象检查调用顺序，不需要每次都启动 Gazebo。

## 6. `ur7e_gazebo`：物理世界和 Gazebo 控制链

目录：`src/ur7e_gazebo/`

### 6.1 包级文件

| 文件               | 作用                                                               |
| ------------------ | ------------------------------------------------------------------ |
| `package.xml`    | 声明 Gazebo、ros_gz、gz_ros2_control、控制器和其他三个项目包的依赖 |
| `CMakeLists.txt` | 安装 world、config、launch、辅助脚本和 README                      |
| `README.md`      | 一键启动、三终端启动、场景坐标和物理边界说明                       |

### 6.2 Gazebo 世界

#### `worlds/ur7e_workspace.sdf`

定义物理世界：

- Bullet Featherstone 物理引擎；
- 1 ms 仿真步长、实时倍率 1.0；
- 地面、灯光和场景颜色；
- 固定桌面、后墙、储物箱和中间隔断；
- 质量 `0.08 kg`、摩擦系数 `1.0` 的动态目标方块；
- 固定在工作单元右前上方的 RGB-D 相机，输出 640×480 彩色图、深度图和内参；
- Gazebo Sensors 系统使用 Ogre2，以 15 Hz 更新相机。

使用 Bullet Featherstone 而不是 DART，是因为当前 Gazebo Harmonic 的 DART 后端无法
创建 Robotiq 所需的 mimic 约束。

### 6.3 Gazebo 配置

#### `config/gazebo_initial_positions.yaml`

机器人生成时六个 UR7e 关节的初始值，即严格向下 READY。这样可以避免机器人从零位
自由落下或先执行一段很长的恢复运动。

#### `config/gazebo_controllers.yaml`

Gazebo 专用 ros2_control 配置：

- 控制器更新频率 250 Hz；
- 标准 JointTrajectoryController 使用项目原有公开名称；
- UR7e 六关节轨迹容差；
- Robotiq 主动关节轨迹容差；
- 保持与 MoveIt/Mock 相同的 Action 名称。

#### `config/workspace_pick_place_gazebo.yaml`

覆盖通用抓取任务中的物理仿真参数：

- `use_sim_time: true`；
- 在回 READY 前先建立 MoveIt 静态场景；
- Gazebo 目标方块和放置位置；
- 抓取 TCP 偏移；
- 适合 35 mm 实体方块的 `0.37 rad` 夹爪位置；
- Gazebo 位置接口对应的轨迹起点容差。

### 6.4 Gazebo Launch 文件

#### `launch/ur7e_gazebo.launch.py`

模块化基础启动：

1. 启动 Gazebo 世界；
2. 以 `use_gazebo:=true` 展开组合 Xacro；
3. 启动 `robot_state_publisher`；
4. 桥接 `/clock` 和 `/world/default/set_pose`；
5. 将 RGB-D 彩色图、深度图和相机内参桥接到 ROS 2；
6. 发布 `base_link → camera_link → optical frame` 静态 TF；
7. 将机器人生成到 Gazebo；
8. 机器人生成成功后加载三个控制器。

#### `launch/workspace_pick_place_gazebo.launch.py`

Gazebo 任务启动：

1. 调用 `set_entity_pose` 把物理方块重置到抓取位置；
2. 检查重置程序退出码；
3. 加载 MoveIt、通用规划和 Gazebo 专用抓取参数；
4. 只有重置成功才启动 `workspace_pick_place_demo`。

#### `launch/ur7e_gazebo_demo.launch.py`

一键启动总入口：

```text
ur7e_gazebo.launch.py
→ wait_for_controllers
→ ur7e_moveit.launch.py
→ 等待 4 秒
→ workspace_pick_place_gazebo.launch.py（run_demo=true 时）
```

对外参数：`gui`、`paused`、`launch_rviz`、`run_demo`、`use_sim_time`。

### 6.5 辅助程序与测试

#### `scripts/wait_for_controllers.py`

轮询 `/controller_manager/list_controllers`，直到以下三个控制器均为 `active`：

- `joint_state_broadcaster`；
- `scaled_joint_trajectory_controller`；
- `gripper_controller`。

它使用墙钟时间，因此 Gazebo 暂停时仍能正确等待和超时。其退出码决定一键 launch 是否
继续启动 MoveIt。

#### `test/test_gazebo_files.py`

静态集成检查：

- Mock 和 Gazebo ros2_control 分支互斥；
- Gazebo 中只存在一个硬件系统；
- 控制器名称和类型保持兼容；
- 世界中固定物体、动态方块、质量、摩擦和物理引擎正确；
- Gazebo 抓取参数符合实体接触几何。

## 7. 运行时文件关系

### 7.1 模型和配置怎样汇合

```text
ur7e_robotiq_2f_85.urdf.xacro
        │
        ├── robot_state_publisher → /tf、/tf_static、robot_description
        ├── MoveItConfigsBuilder → MoveItPy / move_group / RViz
        └── Gazebo create → 物理机器人模型

ur7e_robotiq.srdf
        └── MoveItConfigsBuilder → 规划组、TCP、READY、碰撞语义

kinematics.yaml + joint_limits.yaml + ompl_planning.yaml
        └── MoveItConfigsBuilder → 逆运动学和规划流水线

moveit_controllers.yaml
        └── MoveIt → FollowJointTrajectory Action 名称和关节映射
```

### 7.2 一条机械臂运动命令的数据流

```text
workspace_pick_place_demo.py
        │ 调用 move_to_pose()
        ▼
UR7eMoveItClient
        │ 读取 /joint_states，设置当前规划起点
        │ 通过 KDL 将 TCP 目标转换为关节目标
        │ 通过 OMPL/RRTConnect 搜索无碰撞轨迹
        │ 三次候选中选择关节路径较短者
        ▼
MoveIt 轨迹执行器
        │ FollowJointTrajectory Action
        ▼
scaled_joint_trajectory_controller
        │ position 命令
        ├── Mock GenericSystem
        └── gz_ros2_control → Gazebo 关节物理
        │
        ▼
/joint_states
        │
        ▼
robot_state_publisher → /tf → MoveIt / RViz / 任务节点
```

### 7.3 一条夹爪命令的数据流

```text
GripperClient.move_to(0.37)
        │
        ▼
/gripper_controller/follow_joint_trajectory
        │
        ▼
gripper_controller
        │ 控制 robotiq_85_left_knuckle_joint
        ▼
URDF mimic 关系
        │
        ▼
其余五个夹爪关节同步开合
```

## 8. MoveIt 场景与 Gazebo 世界的关系

当前物理抓取同时维护两个不同的世界：

| 世界                 | 负责什么                                   | 由什么文件创建               |
| -------------------- | ------------------------------------------ | ---------------------------- |
| MoveIt PlanningScene | 路径碰撞检查、附着物体、RViz 显示          | `SceneManager` + 任务 YAML |
| Gazebo 世界          | 重力、刚体动力学、接触、摩擦、方块真实运动 | `ur7e_workspace.sdf`       |

例如 `SceneManager.attach_object()` 只是在 MoveIt 中告诉规划器：“搬运时要把方块看作
夹爪的一部分”。它不会把 Gazebo 方块粘到夹爪上。Gazebo 方块能被抬起，是因为两侧
指尖碰撞体、摩擦系数、夹爪位置和控制器共同产生了真实夹持。

两套场景的物体尺寸和位置必须保持一致，否则可能出现：

- MoveIt 认为无碰撞，但 Gazebo 中已经碰到桌面；
- RViz 方块跟着夹爪移动，但 Gazebo 方块实际已经滑落；
- Gazebo 中存在障碍物，但 MoveIt 不知道它，规划路径直接穿过。

## 9. 主要 ROS 节点与接口

| 节点/进程                     | 主要作用                                           |
| ----------------------------- | -------------------------------------------------- |
| `robot_state_publisher`     | 根据 URDF 和`/joint_states` 发布 TF              |
| `controller_manager`        | 管理关节状态广播器和两个轨迹控制器                 |
| `move_group`                | 为 RViz 等外部客户端提供 MoveIt 接口               |
| `rviz2_moveit`              | 显示机器人、规划轨迹和 PlanningScene               |
| Gazebo`gz sim`              | 物理仿真世界                                       |
| `clock_bridge`              | 将 Gazebo`/clock` 桥接到 ROS，并桥接方块复位服务 |
| `rgbd_camera_bridge`        | 将 Gazebo RGB-D 数据桥接为 ROS 2 `sensor_msgs`      |
| `color_cube_detector`       | 识别三色方块并发布 `base_link` 下的坐标             |
| 三个相机静态 TF 节点       | 发布相机安装坐标系和两个 optical frame              |
| `workspace_pick_place_demo` | 执行一次抓取任务，完成后退出                       |
| MoveItPy 内部节点             | 在任务进程内进行状态监视、规划和轨迹执行           |

主要接口：

| 类型    | 名称                                                            | 方向/用途                          |
| ------- | --------------------------------------------------------------- | ---------------------------------- |
| Topic   | `/joint_states`                                               | 控制系统 → MoveIt、TF、RViz       |
| Topic   | `/tf`、`/tf_static`                                         | robot_state_publisher → 全系统    |
| Topic   | `/planning_scene`                                             | SceneManager → move_group、RViz   |
| Topic   | `/clock`                                                      | Gazebo → ROS 仿真时间             |
| Topic   | `/camera/color/image_raw`                                     | Gazebo RGB-D → 视觉节点           |
| Topic   | `/camera/depth/image_raw`                                     | Gazebo RGB-D → 视觉节点           |
| Topic   | `/camera/color/camera_info`                                   | Gazebo RGB-D → 视觉节点           |
| Topic   | `/color_cube_detector/detections/*/center`                    | 视觉节点 → 任务节点               |
| Topic   | `/color_cube_detector/debug_image`                            | 带识别框和坐标的调试图            |
| Action  | `/scaled_joint_trajectory_controller/follow_joint_trajectory` | MoveIt → UR7e 控制器              |
| Action  | `/gripper_controller/follow_joint_trajectory`                 | GripperClient/MoveIt → 夹爪控制器 |
| Service | `/controller_manager/list_controllers`                        | 查询控制器 active 状态             |
| Service | `/world/default/set_pose`                                     | 重置 Gazebo 动态方块               |

## 10. 两种典型启动链路

### 10.1 Mock Hardware

终端 1：

```bash
ros2 launch ur7e_description ur7e_mock_control.launch.py
```

终端 2：

```bash
ros2 launch ur7e_moveit_config ur7e_moveit.launch.py
```

终端 3：

```bash
ros2 launch ur7e_motion workspace_pick_place_demo.launch.py
```

关系：

```text
ur7e_mock_control.launch.py
  → Xacro + robot_state_publisher + ros2_control_node + controllers

ur7e_moveit.launch.py
  → move_group + RViz

workspace_pick_place_demo.launch.py
  → MoveItPy 参数 + Python 抓取任务
```

### 10.2 Gazebo 一键运行

```bash
ros2 launch ur7e_gazebo ur7e_gazebo_demo.launch.py
```

关系：

```text
ur7e_gazebo_demo.launch.py
  ├── ur7e_gazebo.launch.py
  │     ├── Gazebo world
  │     ├── Gazebo 模式 Xacro
  │     ├── robot_state_publisher
  │     ├── ros_gz_bridge
  │     ├── create robot
  │     └── controller spawner
  ├── wait_for_controllers.py
  ├── ur7e_moveit.launch.py
  │     ├── move_group
  │     └── RViz
  └── workspace_pick_place_gazebo.launch.py
        ├── reset workspace_target
        └── workspace_pick_place_demo
```

## 11. 修改需求时应该找哪个文件

| 想修改的内容              | 优先修改的位置                                              |
| ------------------------- | ----------------------------------------------------------- |
| 夹爪安装方向、TCP 位置    | `ur7e_robotiq_2f_85.urdf.xacro`                           |
| READY 关节角              | `fixed_task.py`、SRDF 命名状态、默认参数、Gazebo 初始位置 |
| MoveIt 速度/加速度        | `joint_limits.yaml`、`motion_planning.yaml`             |
| 更换规划器或规划时间      | `motion_planning.yaml`、`ompl_planning.yaml`            |
| 控制器 Action 映射        | `moveit_controllers.yaml`                                 |
| Mock 控制器参数           | `mock_controllers.yaml`                                   |
| Gazebo 控制器容差/频率    | `gazebo_controllers.yaml`                                 |
| Gazebo 桌面或方块物理参数 | `ur7e_workspace.sdf`                                      |
| MoveIt 场景尺寸/抓取点    | 对应任务 YAML                                               |
| Gazebo 相机位置/成像参数   | `ur7e_workspace.sdf` + `ur7e_gazebo.launch.py`              |
| 颜色阈值和识别范围        | `ur7e_vision/config/color_cube_detector.yaml`                 |
| 修改三维定位算法          | `ur7e_vision/color_detection.py`、`color_cube_detector.py`   |
| 新增机械臂高层动作        | `moveit_client.py`                                        |
| 新增夹爪动作              | `gripper_client.py`                                       |
| 新增场景几何操作          | `scene_manager.py`                                        |
| 修改抓取步骤顺序          | `workspace_pick_place_demo.py`                            |
| 修改一键启动顺序          | `ur7e_gazebo_demo.launch.py`                              |

修改机器人模型、控制器、SRDF 或安装资源后应重新运行 `colcon build`。修改任务代码后也
建议重新构建并运行测试，确认 Python 入口和安装空间仍一致。

## 12. 推荐阅读顺序

如果目标是理解当前完整抓取任务，建议按以下顺序阅读：

1. `workspace_pick_place_demo.py`：先看整个任务要做什么；
2. `moveit_client.py`：理解机械臂每一步怎样规划和执行；
3. `gripper_client.py`：理解夹爪 Action；
4. `scene_manager.py`：理解碰撞场景、附着和触碰许可；
5. `ur7e_robotiq_2f_85.urdf.xacro`：理解机器人结构与控制接口；
6. `ur7e_robotiq.srdf` 和 MoveIt 四个 YAML：理解规划语义；
7. `ur7e_gazebo.launch.py` 与 `ur7e_workspace.sdf`：理解物理仿真；
8. `ur7e_gazebo_demo.launch.py`：最后理解整个系统如何按顺序组合启动。

阅读时可以把一次任务始终理解为：

```text
任务决定“下一步做什么”
→ MoveIt 决定“怎样无碰撞地到达”
→ ros2_control 决定“轨迹发给哪些关节”
→ Mock/Gazebo 决定“关节状态怎样变化”
→ /joint_states 和 /tf 把结果反馈给整个系统
```
