"""启动 UR7e + Robotiq 的 MoveIt move_group，以及可选的 RViz。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """加载本项目模型/规划配置，并启动 MoveIt 的核心服务节点。"""
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # MoveItConfigsBuilder 会收集本包中的 SRDF、运动学、关节限位、OMPL 与控制器映射，
    # 并生成可直接传给 move_group、RViz 或 MoveItPy 的参数字典。
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

    # move_group 是 MoveIt 的 ROS 服务/Action 节点：RViz MotionPlanning 面板通过它做
    # 交互式规划。Python 任务使用 MoveItPy 直接规划，但仍共享相同模型和 PlanningScene。
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                # 发布模型与 SRDF 话题，允许后启动的 MoveIt/RViz 节点获取同一份描述。
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # RViz 仅用于可视化与手动交互；launch_rviz:=false 时保留 move_group 但不启动 GUI。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=[
            # 使用官方 ur_moveit_config 提供的 MotionPlanning 面板布局。
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
            ),
        ],
        # RViz 要自行理解机器人与规划场景，因此也需要与 move_group 对应的模型/规划参数。
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            # Gazebo 模式必须为 true，确保 MoveIt/RViz 使用 /clock；Mock 模式默认 false。
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with the MoveIt MotionPlanning panel",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time from /clock",
            ),
            move_group,
            rviz,
        ]
    )
