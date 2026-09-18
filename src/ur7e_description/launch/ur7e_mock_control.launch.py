"""Start UR7e and Robotiq 2F-85 with ros2_control Mock Hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Publish the combined model and activate its three required controllers."""
    ur_type = LaunchConfiguration("ur_type")
    description_file = PathJoinSubstitution(
        [
            FindPackageShare("ur7e_description"),
            "urdf",
            "ur7e_robotiq_2f_85.urdf.xacro",
        ]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ur7e_description"), "config", "mock_controllers.yaml"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    description_file,
                    " ur_type:=",
                    ur_type,
                    " name:=ur7e_robotiq_2f_85",
                    " use_mock_hardware:=true",
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "scaled_joint_trajectory_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur7e",
                choices=["ur7e"],
                description="UR model used by this project",
            ),
            robot_state_publisher,
            control_node,
            controller_spawner,
        ]
    )
