"""One-command Gazebo, MoveIt, RViz and physical pick-place demo."""

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
    """Wait for controllers before starting MoveIt and the optional task."""
    gazebo_share = Path(get_package_share_directory('ur7e_gazebo'))
    moveit_share = Path(get_package_share_directory('ur7e_moveit_config'))
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')
    launch_rviz = LaunchConfiguration('launch_rviz')
    run_demo = LaunchConfiguration('run_demo')
    use_sim_time = LaunchConfiguration('use_sim_time')

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
    wait_for_controllers = Node(
        package='ur7e_gazebo',
        executable='wait_for_controllers',
        output='screen',
        parameters=[{'timeout_seconds': 90.0, 'use_sim_time': False}],
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / 'launch' / 'ur7e_moveit.launch.py')
        ),
        launch_arguments={
            'launch_rviz': launch_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )
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
        if event.returncode != 0:
            return [
                LogInfo(msg='ERROR: Gazebo controllers failed to become active'),
                EmitEvent(event=Shutdown(reason='controller startup failed')),
            ]
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
