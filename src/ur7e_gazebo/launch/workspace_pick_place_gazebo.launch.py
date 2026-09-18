"""Reset the physical cube and run the Gazebo workcell pick-place task."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    """Start the task only after its physical target has been reset."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_share = Path(get_package_share_directory('ur7e_gazebo'))
    motion_share = Path(get_package_share_directory('ur7e_motion'))
    task_config = gazebo_share / 'config' / 'workspace_pick_place_gazebo.yaml'
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
    # set_entity_pose is a command-line utility rather than an rclcpp node;
    # ExecuteProcess avoids appending ROS-specific remapping arguments.
    reset_target = ExecuteProcess(
        cmd=[
            str(Path(get_package_share_directory('ros_gz_sim')).parents[1]
                / 'lib' / 'ros_gz_sim' / 'set_entity_pose'),
            '--name', 'workspace_target',
            '--type', '2',
            '--pos', '0.16', '0.47', '0.331',
            '--quat', '0.0', '0.0', '0.0', '1.0',
        ],
        output='screen',
    )
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

    def start_after_reset(event, _context):
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
            reset_target,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=reset_target,
                    on_exit=start_after_reset,
                )
            ),
        ]
    )
