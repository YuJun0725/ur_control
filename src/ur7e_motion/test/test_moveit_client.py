"""Tests for planning and execution behavior in the MoveIt client."""

from types import SimpleNamespace

import pytest

from ur7e_motion import MotionPlanningError, UR7eMoveItClient
from ur7e_motion.moveit_client import (
    _joint_path_length,
    PLANNING_CANDIDATES,
)


class FakeTrajectory:
    """Expose a trajectory message with configurable joint positions."""

    def __init__(self, positions):
        """Build fake joint trajectory points."""
        points = [SimpleNamespace(positions=values) for values in positions]
        self.message = SimpleNamespace(
            joint_trajectory=SimpleNamespace(points=points)
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

    def warning(self, message):
        """Record a warning message."""
        self.warnings.append(message)

    def info(self, message):
        """Accept informational logs that are irrelevant to this test."""
        pass


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
