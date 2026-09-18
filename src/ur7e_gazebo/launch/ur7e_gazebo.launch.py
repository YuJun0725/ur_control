"""Start Gazebo Harmonic with the UR7e, Robotiq gripper and controllers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gazebo_include(context):
    package_share = Path(get_package_share_directory('ur7e_gazebo'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))
    gui = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes')
    paused = LaunchConfiguration('paused').perform(context).lower() in (
        '1', 'true', 'yes'
    )
    options = []
    if not paused:
        options.append('-r')
    if not gui:
        options.append('-s')
    options.append(str(package_share / 'worlds' / 'ur7e_workspace.sdf'))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_share / 'launch' / 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ' '.join(options),
                'on_exit_shutdown': 'true',
            }.items(),
        )
    ]


def generate_launch_description():
    """Spawn the robot and activate the existing public controller names."""
    package_share = Path(get_package_share_directory('ur7e_gazebo'))
    description_share = Path(get_package_share_directory('ur7e_description'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    controllers_file = package_share / 'config' / 'gazebo_controllers.yaml'
    initial_positions_file = (
        package_share / 'config' / 'gazebo_initial_positions.yaml'
    )
    description_file = (
        description_share / 'urdf' / 'ur7e_robotiq_2f_85.urdf.xacro'
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(
                [
                    'xacro ',
                    str(description_file),
                    ' ur_type:=ur7e',
                    ' name:=ur7e_robotiq_2f_85',
                    ' use_mock_hardware:=false',
                    ' use_gazebo:=true',
                    ' initial_positions_file:=',
                    str(initial_positions_file),
                    ' gazebo_controllers_file:=',
                    str(controllers_file),
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ur7e_robotiq',
        output='screen',
        arguments=[
            '-world', 'default',
            '-topic', '/robot_description',
            '-name', 'ur7e_robotiq',
            '-allow_renaming', 'false',
        ],
    )
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_ur7e_controllers',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'scaled_joint_trajectory_controller',
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '10',
            '--switch-timeout', '20',
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gui', default_value='true', description='Start Gazebo GUI'
            ),
            DeclareLaunchArgument(
                'paused',
                default_value='false',
                description='Start the simulation paused',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo /clock topic',
            ),
            OpaqueFunction(function=_gazebo_include),
            robot_state_publisher,
            clock_bridge,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[controller_spawner],
                )
            ),
        ]
    )
