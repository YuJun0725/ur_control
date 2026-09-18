"""Launch the fixed UR7e demonstration task."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    """Load MoveIt parameters and start the fixed task node."""
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur7e_robotiq",
            package_name="ur7e_moveit_config",
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )

    motion_config_path = (
        Path(get_package_share_directory("ur7e_motion"))
        / "config"
        / "motion_planning.yaml"
    )
    with motion_config_path.open(encoding="utf-8") as config_file:
        motion_config = yaml.safe_load(config_file)

    pause_argument = DeclareLaunchArgument(
        "pause_seconds",
        default_value="1.0",
        description="Pause after each relative motion, in seconds",
    )
    task_node = Node(
        package="ur7e_motion",
        executable="fixed_task",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            motion_config,
            {
                "pause_seconds": ParameterValue(
                    LaunchConfiguration("pause_seconds"), value_type=float
                )
            },
        ],
    )

    return LaunchDescription([pause_argument, task_node])
