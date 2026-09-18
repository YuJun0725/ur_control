"""Tests for the synchronous Robotiq gripper action client."""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import pytest

from ur7e_motion import (
    GripperClient,
    GripperCommandFailedError,
    GripperCommandRejectedError,
    GripperCommandTimeoutError,
    GripperServerUnavailableError,
)


class FakeLogger:
    """Accept informational messages emitted by the client."""

    def info(self, message):
        """Ignore the message while preserving the logger interface."""
        pass


class FakeFuture:
    """Provide the subset of the rclpy Future API used by the client."""

    def __init__(self, result, *, done=True):
        """Store a deterministic result and completion state."""
        self._result = result
        self._done = done

    def done(self):
        """Return whether the fake operation completed."""
        return self._done

    def result(self):
        """Return the configured fake result."""
        return self._result


class FakeGoalHandle:
    """Represent an accepted or rejected trajectory action goal."""

    def __init__(self, result_future, *, accepted=True):
        """Store acceptance and result behavior."""
        self.accepted = accepted
        self.result_future = result_future
        self.cancelled = False

    def get_result_async(self):
        """Return the configured result future."""
        return self.result_future

    def cancel_goal_async(self):
        """Record cancellation after an execution timeout."""
        self.cancelled = True


class FakeActionClient:
    """Record trajectory goals without requiring a controller."""

    def __init__(self, goal_handle, *, server_available=True):
        """Configure discovery and goal behavior."""
        self.goal_handle = goal_handle
        self.server_available = server_available
        self.goals = []
        self.destroyed = False

    def wait_for_server(self, *, timeout_sec):
        """Return the configured discovery result."""
        return self.server_available

    def send_goal_async(self, goal):
        """Record the goal and return its handle immediately."""
        self.goals.append(goal)
        return FakeFuture(self.goal_handle)

    def destroy(self):
        """Record resource cleanup."""
        self.destroyed = True


def _successful_result():
    return SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            error_code=FollowJointTrajectory.Result.SUCCESSFUL,
            error_string='',
        ),
    )


def _client(action_client):
    client = GripperClient.__new__(GripperClient)
    client._node = object()
    client._logger = FakeLogger()
    client._action_type = FollowJointTrajectory
    client._open_position = 0.0
    client._closed_position = 0.8
    client._server_wait_seconds = 5.0
    client._result_timeout_padding = 5.0
    client._action_name = '/gripper_controller/follow_joint_trajectory'
    client._action_client = action_client
    client._wait_for_future = lambda future, timeout: future.done()
    return client


def test_open_and_close_send_time_parameterized_trajectory_goals():
    """Convenience methods send the expected endpoint and duration."""
    result_future = FakeFuture(_successful_result())
    action_client = FakeActionClient(FakeGoalHandle(result_future))
    client = _client(action_client)

    client.open(duration=1.5)
    client.close(duration=2.0)

    open_point = action_client.goals[0].trajectory.points[0]
    close_point = action_client.goals[1].trajectory.points[0]
    assert action_client.goals[0].trajectory.joint_names == [
        'robotiq_85_left_knuckle_joint'
    ]
    assert list(open_point.positions) == [0.0]
    assert open_point.time_from_start.sec == 1
    assert open_point.time_from_start.nanosec == 500_000_000
    assert list(close_point.positions) == [0.8]
    assert close_point.time_from_start.sec == 2


@pytest.mark.parametrize('position', [-0.01, 0.81, float('nan'), True])
def test_invalid_position_is_rejected_without_contacting_controller(position):
    """Unsafe target values never reach the action server."""
    action_client = FakeActionClient(
        FakeGoalHandle(FakeFuture(_successful_result()))
    )
    client = _client(action_client)

    with pytest.raises(ValueError, match='position'):
        client.move_to(position)

    assert action_client.goals == []


@pytest.mark.parametrize('duration', [0.0, -1.0, float('inf'), True])
def test_invalid_duration_is_rejected(duration):
    """A command must have a positive finite travel duration."""
    client = _client(
        FakeActionClient(FakeGoalHandle(FakeFuture(_successful_result())))
    )

    with pytest.raises(ValueError, match='duration'):
        client.move_to(0.4, duration)


def test_missing_action_server_is_reported():
    """Controller discovery failure has a specific exception type."""
    client = _client(
        FakeActionClient(
            FakeGoalHandle(FakeFuture(_successful_result())),
            server_available=False,
        )
    )

    with pytest.raises(GripperServerUnavailableError):
        client.close()


def test_rejected_goal_is_reported():
    """An action goal rejection stops the task immediately."""
    goal_handle = FakeGoalHandle(
        FakeFuture(_successful_result()), accepted=False
    )
    client = _client(FakeActionClient(goal_handle))

    with pytest.raises(GripperCommandRejectedError):
        client.close()


def test_unsuccessful_controller_result_is_reported():
    """An aborted trajectory becomes a gripper control exception."""
    failed_result = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(error_code=-4, error_string='goal tolerance'),
    )
    client = _client(
        FakeActionClient(FakeGoalHandle(FakeFuture(failed_result)))
    )

    with pytest.raises(GripperCommandFailedError, match='goal tolerance'):
        client.close()


def test_execution_timeout_cancels_the_goal():
    """A controller that never finishes is asked to cancel its goal."""
    result_future = FakeFuture(None, done=False)
    goal_handle = FakeGoalHandle(result_future)
    client = _client(FakeActionClient(goal_handle))

    with pytest.raises(GripperCommandTimeoutError):
        client.close()

    assert goal_handle.cancelled


def test_destroy_releases_action_client():
    """Explicit cleanup releases the rclpy action client."""
    action_client = FakeActionClient(
        FakeGoalHandle(FakeFuture(_successful_result()))
    )
    client = _client(action_client)

    client.destroy()

    assert action_client.destroyed
