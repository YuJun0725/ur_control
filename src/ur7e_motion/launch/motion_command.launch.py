"""Launch one UR7e MoveItPy motion command."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    """Create a launch description for one parameterized motion."""
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

    mode_argument = DeclareLaunchArgument(
        "mode",
        default_value="joint",
        description="Motion mode: joint or pose",
    )
    joint_positions_argument = DeclareLaunchArgument(
        "joint_positions",
        default_value=(
            "[1.54, -1.62, 1.4, -1.350796327, "
            "-1.570796326589793, -0.030796327]"
        ),
        description="Six UR7e joint positions in radians",
    )
    translation_argument = DeclareLaunchArgument(
        "translation",
        default_value="[0.0, 0.0, 0.05]",
        description=(
            "robotiq_tcp translation [dx, dy, dz] in base_link, in metres"
        ),
    )

    motion_node = Node(
        package="ur7e_motion",
        executable="motion_command",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            motion_config,
            {
                "mode": LaunchConfiguration("mode"),
                "joint_positions": ParameterValue(
                    LaunchConfiguration("joint_positions")
                ),
                "translation": ParameterValue(LaunchConfiguration("translation")),
            },
        ],
    )

    return LaunchDescription(
        [
            mode_argument,
            joint_positions_argument,
            translation_argument,
            motion_node,
        ]
    )
