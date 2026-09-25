"""一键启动 Gazebo、MoveIt、RViz 和可选的物理抓取演示。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """按依赖顺序启动：Gazebo/控制器 → MoveIt/RViz → 可选抓取任务。"""
    gazebo_share = Path(get_package_share_directory('ur7e_gazebo'))
    moveit_share = Path(get_package_share_directory('ur7e_moveit_config'))
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')
    launch_rviz = LaunchConfiguration('launch_rviz')
    run_demo = LaunchConfiguration('run_demo')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 第一阶段：物理世界、机器人模型、/clock 桥和控制器。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / 'launch' / 'ur7e_gazebo.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'paused': paused,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    # 轮询三个控制器是否都为 active；使用墙钟时间，因而 Gazebo paused 时也不会卡住。
    wait_for_controllers = Node(
        package='ur7e_gazebo',
        executable='wait_for_controllers',
        output='screen',
        parameters=[{'timeout_seconds': 90.0, 'use_sim_time': False}],
    )
    # 第二阶段：控制器就绪后才启动 MoveIt 和 RViz，避免其初始状态读取失败。
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / 'launch' / 'ur7e_moveit.launch.py')
        ),
        launch_arguments={
            'launch_rviz': launch_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    # 第三阶段：run_demo:=true 时，调用会先重置物理方块的抓取 launch。
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                gazebo_share
                / 'launch'
                / 'workspace_pick_place_gazebo.launch.py'
            )
        ),
        condition=IfCondition(run_demo),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    def start_after_controllers(event, _context):
        # 控制器等待节点的退出码是启动门槛：失败时让整个一键启动失败，而不是继续
        # 启动一个无法控制机器人的 MoveIt/RViz 环境。
        if event.returncode != 0:
            return [
                LogInfo(msg='ERROR: Gazebo controllers failed to become active'),
                EmitEvent(event=Shutdown(reason='controller startup failed')),
            ]
        # 留出少量时间让 move_group 建立 PlanningScene 和控制器 Action 连接。
        return [moveit, TimerAction(period=4.0, actions=[demo])]

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('paused', default_value='false'),
            DeclareLaunchArgument('launch_rviz', default_value='true'),
            DeclareLaunchArgument('run_demo', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            gazebo,
            wait_for_controllers,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_controllers,
                    on_exit=start_after_controllers,
                )
            ),
        ]
    )
