"""Tests for planning and execution behavior in the MoveIt client."""

from contextlib import nullcontext
from types import SimpleNamespace

import moveit.core.robot_state as robot_state_module

import pytest

from ur7e_motion import MotionPlanningError, UR7eMoveItClient
from ur7e_motion.moveit_client import (
    _joint_path_length,
    _tcp_path_length,
    PLANNING_CANDIDATES,
)


class FakeTrajectory:
    """Expose a trajectory message with configurable joint positions."""

    def __init__(self, positions):
        """Build fake joint trajectory points."""
        points = [SimpleNamespace(positions=values) for values in positions]
        joint_count = len(positions[0]) if positions else 0
        self.message = SimpleNamespace(
            joint_trajectory=SimpleNamespace(
                points=points,
                joint_names=[f'joint_{i}' for i in range(joint_count)],
            )
        )

    def get_robot_trajectory_msg(self):
        """Return the fake ROS trajectory message."""
        return self.message


class FakePlanResult:
    """Represent a truthy or falsey MoveIt planning result."""

    def __init__(
        self,
        succeeded,
        positions=((0.0, 0.0), (1.0, 0.0)),
        error_code='INVALID_MOTION_PLAN',
    ):
        """Store the outcome and fields consumed by the client."""
        self.succeeded = succeeded
        self.error_code = error_code
        self.planning_time = 0.01
        self.trajectory = FakeTrajectory(positions)

    def __bool__(self):
        """Return whether planning succeeded."""
        return self.succeeded


class FakeLogger:
    """Record client log messages."""

    def __init__(self):
        """Create an empty warning log."""
        self.warnings = []
        self.infos = []

    def warning(self, message):
        """Record a warning message."""
        self.warnings.append(message)

    def info(self, message):
        """Record candidate metrics for assertions."""
        self.infos.append(message)


def _client_with_results(results):
    client = UR7eMoveItClient.__new__(UR7eMoveItClient)
    client._logger = FakeLogger()
    client._plan_parameters = object()
    client._arm = SimpleNamespace(plan=lambda **kwargs: results.pop(0))
    executions = []
    client._moveit = SimpleNamespace(
        execute=lambda trajectory, controllers: executions.append(trajectory)
        or SimpleNamespace(__bool__=lambda self: True)
    )
    return client, executions


def test_planning_selects_shortest_valid_candidate():
    """Invalid paths are ignored and the shortest valid path is executed."""
    long_result = FakePlanResult(
        True, positions=((0.0, 0.0), (3.0, 4.0))
    )
    short_result = FakePlanResult(
        True, positions=((0.0, 0.0), (0.0, 2.0))
    )
    client, executions = _client_with_results(
        [FakePlanResult(False), long_result, short_result]
    )

    client._plan_and_execute()

    assert executions == [short_result.trajectory]
    assert len(client._logger.warnings) == 1


def test_planning_failure_is_reported_after_all_attempts():
    """Repeated invalid paths stop without sending any trajectory."""
    client, executions = _client_with_results(
        [FakePlanResult(False) for _ in range(PLANNING_CANDIDATES)]
    )

    with pytest.raises(MotionPlanningError, match='no valid trajectory'):
        client._plan_and_execute()

    assert executions == []


def test_joint_path_length_sums_euclidean_segment_lengths():
    """Trajectory scoring accumulates all joint-space segments."""
    trajectory = FakeTrajectory(
        ((0.0, 0.0), (3.0, 4.0), (3.0, 6.0))
    )

    assert _joint_path_length(trajectory) == pytest.approx(7.0)


class FakeFKState:
    """Map two fake joints to TCP x/y for metric tests."""

    def __init__(self):
        self.joint_positions = {'joint_0': 0.0, 'joint_1': 0.0}

    def update(self):
        pass

    def get_pose(self, link_name):
        assert link_name == 'robotiq_tcp'
        return SimpleNamespace(
            position=SimpleNamespace(
                x=self.joint_positions['joint_0'],
                y=self.joint_positions['joint_1'],
                z=0.0,
            )
        )


def test_tcp_path_length_uses_forward_kinematics():
    trajectory = FakeTrajectory(
        ((0.0, 0.0), (3.0, 4.0), (3.0, 6.0))
    )

    assert _tcp_path_length(trajectory, FakeFKState()) == pytest.approx(7.0)


@pytest.mark.parametrize('valid', [True, False])
def test_nearby_ik_is_used_only_when_goal_state_is_collision_free(
    monkeypatch, valid
):
    calls = []

    class FakeIKState:
        def __init__(self):
            self.joint_positions = {}

        def update(self):
            pass

        def set_from_ik(self, group, pose, tip, timeout):
            calls.append(('ik', dict(self.joint_positions), group, tip))
            return True

    ik_state = FakeIKState()
    monkeypatch.setattr(
        robot_state_module, 'RobotState', lambda model: ik_state
    )
    client = UR7eMoveItClient.__new__(UR7eMoveItClient)
    client._moveit = SimpleNamespace(get_robot_model=lambda: object())
    client._logger = FakeLogger()
    scene = SimpleNamespace(
        is_state_valid=lambda state, group: calls.append(
            ('validity', state, group)
        ) or valid
    )
    client._scene_monitor = SimpleNamespace(
        read_only=lambda: nullcontext(scene)
    )
    goal = SimpleNamespace(
        header=SimpleNamespace(frame_id='base_link'),
        pose=object(),
    )
    current = SimpleNamespace(joint_positions={'shoulder_pan_joint': 0.4})

    result = client._nearest_valid_ik_goal(goal, current)

    assert result is (ik_state if valid else None)
    assert calls[0] == (
        'ik', {'shoulder_pan_joint': 0.4},
        'ur_manipulator', 'robotiq_tcp'
    )
    assert calls[1] == ('validity', ik_state, 'ur_manipulator')


def test_pose_compares_nearby_ik_with_regular_pose_candidates():
    preferred = FakePlanResult(
        True, positions=((0.0, 0.0), (0.0, 1.0))
    )
    regular_long = FakePlanResult(
        True, positions=((0.0, 0.0), (3.0, 4.0))
    )
    regular_short = FakePlanResult(
        True, positions=((0.0, 0.0), (0.0, 2.0))
    )
    client, executions = _client_with_results(
        [preferred, regular_long, regular_short]
    )
    goal = object()
    nearby_goal = object()
    goal_calls = []
    client._nearest_valid_ik_goal = lambda goal, current: nearby_goal
    client._arm.set_goal_state = lambda **kwargs: goal_calls.append(
        ('joint', kwargs['robot_state'])
    ) or True
    client._set_pose_goal = lambda goal: goal_calls.append(('pose', goal))
    client._measure_tcp_path_length = lambda trajectory, current: 0.25

    client._plan_pose_and_execute(
        goal, SimpleNamespace(joint_positions={})
    )

    assert goal_calls == [('joint', nearby_goal), ('pose', goal)]
    assert executions == [preferred.trajectory]
    assert any(
        'TCP length=0.2500 m' in info for info in client._logger.infos
    )


def test_pose_keeps_three_regular_attempts_when_nearby_ik_collides():
    plans = [
        FakePlanResult(False),
        FakePlanResult(True, positions=((0.0, 0.0), (0.0, 2.0))),
        FakePlanResult(True, positions=((0.0, 0.0), (0.0, 3.0))),
    ]
    client, executions = _client_with_results(plans)
    goals = []
    client._nearest_valid_ik_goal = lambda goal, current: None
    client._set_pose_goal = goals.append
    client._measure_tcp_path_length = lambda trajectory, current: None

    client._plan_pose_and_execute(
        'pose-target', SimpleNamespace(joint_positions={})
    )

    assert plans == []
    assert goals == ['pose-target']
    assert len(executions) == 1
    assert len(client._logger.warnings) == 1
