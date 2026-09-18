"""Start MoveIt and RViz for the UR7e with Robotiq 2F-85."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Start move_group with the custom URDF, SRDF and controller mapping."""
    launch_rviz = LaunchConfiguration("launch_rviz")
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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with the MoveIt MotionPlanning panel",
            ),
            move_group,
            rviz,
        ]
    )
