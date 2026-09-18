"""Launch the UR7e workcell collision-scene pick-and-place demo."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    """Load MoveIt and workcell parameters then start one task node."""
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ur7e_robotiq',
            package_name='ur7e_moveit_config',
        )
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
        )
        .to_moveit_configs()
    )

    package_share = Path(get_package_share_directory('ur7e_motion'))
    motion_config_path = package_share / 'config' / 'motion_planning.yaml'
    task_config_path = package_share / 'config' / 'workspace_pick_place_demo.yaml'
    with motion_config_path.open(encoding='utf-8') as config_file:
        motion_config = yaml.safe_load(config_file)

    demo_node = Node(
        package='ur7e_motion',
        executable='workspace_pick_place_demo',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            motion_config,
            str(task_config_path),
        ],
    )
    return LaunchDescription([demo_node])
