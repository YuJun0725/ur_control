"""启动固定目标或 RGB-D 视觉目标驱动的 Gazebo 抓取任务。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    """固定模式先复位；视觉模式从当前相机观测定位方块。"""
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_vision_target = LaunchConfiguration('use_vision_target')
    gazebo_share = Path(get_package_share_directory('ur7e_gazebo'))
    motion_share = Path(get_package_share_directory('ur7e_motion'))
    vision_share = Path(get_package_share_directory('ur7e_vision'))
    task_config = gazebo_share / 'config' / 'workspace_pick_place_gazebo.yaml'
    # workspace_pick_place_demo 使用 MoveItPy；它需要同样的 OMPL/规划参数，
    # 这里读取 ur7e_motion 通用规划 YAML，并与 MoveItConfigsBuilder 的模型参数合并。
    with (motion_share / 'config' / 'motion_planning.yaml').open(
        encoding='utf-8'
    ) as config_file:
        motion_config = yaml.safe_load(config_file)

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ur7e_robotiq',
            package_name='ur7e_moveit_config',
        )
        .planning_pipelines(
            default_planning_pipeline='ompl', pipelines=['ompl']
        )
        .to_moveit_configs()
    )
    # set_entity_pose 是命令行程序，不是 rclcpp 节点；必须用 ExecuteProcess，而不是
    # Node，否则 launch 会为它附加 ROS remapping 参数并导致命令失败。
    # type=2 表示 Gazebo 的 MODEL 实体；位置必须与 Gazebo 场景中的目标方块一致。
    reset_target = ExecuteProcess(
        cmd=[
            str(Path(get_package_share_directory('ros_gz_sim')).parents[1]
                / 'lib' / 'ros_gz_sim' / 'set_entity_pose'),
            '--name', 'workspace_target',
            '--type', '2',
            '--pos', '0.22', '0.47', '0.331',
            '--quat', '0.0', '0.0', '0.0', '1.0',
        ],
        output='screen',
        condition=UnlessCondition(use_vision_target),
    )
    # 复用 Mock/RViz 版本的 Python 任务节点，只通过 Gazebo 专用 YAML 覆盖抓取高度、
    # 夹爪闭合角度及仿真起点容差，因此上层任务接口保持一致。
    demo_node = Node(
        package='ur7e_motion',
        executable='workspace_pick_place_demo',
        name='workspace_pick_place_gazebo',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            motion_config,
            str(task_config),
            {'use_sim_time': use_sim_time},
        ],
    )

    # 视觉模式不会执行固定位置重置；识别器只接收相机图像，任务等待新鲜结果。
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vision_share / 'launch' / 'color_cube_detector.launch.py')
        ),
        condition=IfCondition(use_vision_target),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    vision_demo_node = Node(
        package='ur7e_motion',
        executable='workspace_pick_place_demo',
        name='workspace_pick_place_gazebo',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            motion_config,
            str(task_config),
            {'use_sim_time': use_sim_time, 'use_vision_target': True},
        ],
    )

    def start_after_reset(event, _context):
        # 方块未能复位时直接关闭本次 launch，避免任务对未知初始状态执行抓取。
        if event.returncode != 0:
            return [
                LogInfo(msg='ERROR: failed to reset Gazebo workspace_target'),
                EmitEvent(event=Shutdown(reason='target reset failed')),
            ]
        return [demo_node]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time', default_value='true', description='Use Gazebo time'
            ),
            DeclareLaunchArgument(
                'use_vision_target', default_value='false',
                description='Use the RGB-D estimated red cube pose'
            ),
            reset_target,
            detector,
            TimerAction(
                period=1.0, actions=[vision_demo_node],
                condition=IfCondition(use_vision_target),
            ),
            # 只有 set_entity_pose 正常退出，才创建真正的抓取节点。
            RegisterEventHandler(
                OnProcessExit(
                    target_action=reset_target,
                    on_exit=start_after_reset,
                )
            ),
        ]
    )
