"""启动 Gazebo Harmonic、UR7e + Robotiq 机器人、时钟桥和控制器。"""

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
    """根据 gui/paused 参数组装 Gazebo 启动选项，并包含官方 gz_sim launch。"""
    package_share = Path(get_package_share_directory('ur7e_gazebo'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))
    gui = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes')
    paused = LaunchConfiguration('paused').perform(context).lower() in (
        '1', 'true', 'yes'
    )
    options = []
    # -r：启动后立刻运行仿真；-s：只启动服务器，不显示 Gazebo GUI。
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
    """展开 Xacro，向 Gazebo 生成机器人，并在其后加载三个控制器。"""
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

    # Command 会在 launch 时执行 xacro。关键是 use_gazebo:=true：同一个机器人模型
    # 因此切换到单一 GazeboSimSystem，而不是 Mock Hardware 的两个硬件系统。
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

    # robot_state_publisher 根据 robot_description + /joint_states 发布整棵 /tf 树，
    # 所以 RViz、MoveIt 和任务节点都能得到 base_link → robotiq_tcp 的坐标变换。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    # 桥接 Gazebo 仿真时间；同时桥接 set_pose 服务供重复抓取任务重置动态方块。
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
    # RGB-D 传感器在 Gazebo Transport 中发布数据；parameter_bridge 将三路数据
    # 单向桥接为标准 ROS 2 sensor_msgs。深度图和彩色图已经由同一个 RGB-D
    # 传感器配准，所以共用一份 CameraInfo 和 color optical frame。
    rgbd_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rgbd_camera_bridge',
        output='screen',
        arguments=[
            '/workspace_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/workspace_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            (
                '/workspace_camera/camera_info@'
                'sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ),
        ],
        remappings=[
            ('/workspace_camera/image', '/camera/color/image_raw'),
            ('/workspace_camera/depth_image', '/camera/depth/image_raw'),
            ('/workspace_camera/camera_info', '/camera/color/camera_info'),
        ],
    )

    # 相机是世界中的静态模型，而 base_link 与 Gazebo 世界原点重合。因此这里的
    # 平移和 RPY 必须与 ur7e_workspace.sdf 中 workspace_rgbd_camera 的 pose 一致。
    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        output='screen',
        arguments=[
            '--x', '0.60', '--y', '0.10', '--z', '1.10',
            '--roll', '0.0', '--pitch', '0.844', '--yaw', '2.577',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )
    # ROS 相机光学坐标系：Z 向前、X 向右、Y 向下。RGB-D 的深度和彩色成像
    # 原点重合，因此两个 optical frame 使用同一静态变换。
    camera_color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_optical_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679', '--pitch', '0',
            '--yaw', '-1.57079632679',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )
    camera_depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_optical_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679', '--pitch', '0',
            '--yaw', '-1.57079632679',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_depth_optical_frame',
        ],
    )
    # create 节点从 /robot_description 读取展开后的 URDF/SDF，并生成名为 ur7e_robotiq
    # 的 Gazebo 模型。模型生成成功后才允许加载控制器。
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
    # 这里仅激活控制器；每个控制器的关节列表、频率和容差在
    # config/gazebo_controllers.yaml 中定义。名称保持与 Mock/MoveIt 一致。
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
            # OpaqueFunction 让我们能在 launch 运行期读取布尔参数并拼接 gz_args。
            OpaqueFunction(function=_gazebo_include),
            robot_state_publisher,
            clock_bridge,
            rgbd_camera_bridge,
            camera_mount_tf,
            camera_color_optical_tf,
            camera_depth_optical_tf,
            spawn_robot,
            # 防止 controller_manager 尚未创建就调用 spawner。
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[controller_spawner],
                )
            ),
        ]
    )
