"""Static integration checks for the Gazebo workcell package."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent
DESCRIPTION_XACRO = (
    WORKSPACE_SRC
    / 'ur7e_description'
    / 'urdf'
    / 'ur7e_robotiq_2f_85.urdf.xacro'
)


def _expand_xacro(*arguments: str) -> ET.Element:
    result = subprocess.run(
        ['xacro', str(DESCRIPTION_XACRO), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def test_mock_and_gazebo_ros2_control_are_mutually_exclusive():
    mock = _expand_xacro('use_mock_hardware:=true', 'use_gazebo:=false')
    gazebo = _expand_xacro(
        'use_mock_hardware:=false',
        'use_gazebo:=true',
        'initial_positions_file:='
        + str(PACKAGE_ROOT / 'config' / 'gazebo_initial_positions.yaml'),
        'gazebo_controllers_file:='
        + str(PACKAGE_ROOT / 'config' / 'gazebo_controllers.yaml'),
    )

    assert len(mock.findall('ros2_control')) == 2
    gazebo_systems = gazebo.findall('ros2_control')
    assert len(gazebo_systems) == 1
    assert gazebo_systems[0].findtext('hardware/plugin') == (
        'gz_ros2_control/GazeboSimSystem'
    )
    command_joints = {
        joint.attrib['name']
        for joint in gazebo_systems[0].findall('joint')
        if joint.find('command_interface') is not None
    }
    assert command_joints == {
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
        'robotiq_85_left_knuckle_joint',
    }
    plugins = gazebo.findall('gazebo/plugin')
    assert len(plugins) == 1
    assert plugins[0].attrib['name'] == (
        'gz_ros2_control::GazeboSimROS2ControlPlugin'
    )


def test_controller_names_and_types_preserve_moveit_interfaces():
    with (PACKAGE_ROOT / 'config' / 'gazebo_controllers.yaml').open(
        encoding='utf-8'
    ) as config_file:
        config = yaml.safe_load(config_file)

    manager = config['controller_manager']['ros__parameters']
    assert manager['update_rate'] == 250
    assert manager['scaled_joint_trajectory_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController'
    )
    assert manager['gripper_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController'
    )
    assert config['scaled_joint_trajectory_controller']['ros__parameters'][
        'joints'
    ] == [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]


def test_world_contains_fixed_workcell_and_dynamic_target():
    world_file = PACKAGE_ROOT / 'worlds' / 'ur7e_workspace.sdf'
    root = ET.parse(world_file).getroot()
    world = root.find('world')
    assert world is not None
    assert world.attrib['name'] == 'default'
    physics = world.find('physics')
    assert physics is not None
    assert physics.attrib['type'] == 'bullet'
    assert physics.findtext('max_step_size') == '0.001'
    assert physics.findtext('real_time_factor') == '1.0'
    assert world.findtext(
        "plugin[@name='gz::sim::systems::Physics']/engine/filename"
    ) == 'gz-physics-bullet-featherstone-plugin'

    models = {model.attrib['name']: model for model in world.findall('model')}
    assert {
        'workspace_table',
        'workspace_back_wall',
        'workspace_storage_box',
        'workspace_center_divider',
        'workspace_target',
    }.issubset(models)
    for name in (
        'workspace_table',
        'workspace_back_wall',
        'workspace_storage_box',
        'workspace_center_divider',
    ):
        assert models[name].findtext('static') == 'true'

    target = models['workspace_target']
    assert target.find('static') is None
    assert target.findtext('link/inertial/mass') == '0.08'
    assert target.findtext(
        'link/collision/surface/friction/ode/mu'
    ) == '1.0'
    assert target.findtext(
        'link/collision/surface/bounce/restitution_coefficient'
    ) == '0.0'

    subprocess.run(
        ['gz', 'sdf', '-k', str(world_file)],
        check=True,
        capture_output=True,
        text=True,
    )


def test_gazebo_task_uses_physical_contact_geometry():
    with (
        PACKAGE_ROOT / 'config' / 'workspace_pick_place_gazebo.yaml'
    ).open(encoding='utf-8') as config_file:
        parameters = yaml.safe_load(config_file)['/**']['ros__parameters']

    assert parameters['target_position'] == [0.16, 0.47, 0.331]
    assert parameters['place_position'] == [0.16, 0.32, 0.331]
    assert parameters['grasp_tcp_offset'] == [0.0, 0.0, 0.0]
    assert parameters['lift_translation'] == [0.0, 0.0, 0.10]
    assert parameters['grasp_position'] == 0.37
    assert parameters['static_scene_before_ready'] is True
    assert parameters['trajectory_execution.allowed_start_tolerance'] == 0.1
