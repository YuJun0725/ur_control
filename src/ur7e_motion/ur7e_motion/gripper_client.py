"""Synchronous Python control client for the Robotiq 2F-85 gripper."""

import math
from typing import Any


GRIPPER_ACTION_NAME = '/gripper_controller/follow_joint_trajectory'
GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'
MIN_POSITION = 0.0
MAX_POSITION = 0.8
DEFAULT_OPEN_POSITION = 0.0
DEFAULT_CLOSED_POSITION = 0.8
DEFAULT_DURATION = 2.0


class GripperControlError(RuntimeError):
    """Base exception for gripper command failures."""


class GripperServerUnavailableError(GripperControlError):
    """Raised when the gripper action server cannot be discovered."""


class GripperCommandRejectedError(GripperControlError):
    """Raised when the controller rejects a gripper goal."""


class GripperCommandTimeoutError(GripperControlError):
    """Raised when sending or executing a gripper goal times out."""


class GripperCommandFailedError(GripperControlError):
    """Raised when the controller reports an unsuccessful result."""


class GripperClient:
    """Send blocking, time-parameterized commands to the 2F-85 controller.

    The caller owns the supplied ROS node.  This client spins that node only
    while waiting for action responses, making it convenient for sequential
    task scripts such as a pick-and-place routine.
    """

    def __init__(
        self,
        node: Any,
        *,
        action_name: str = GRIPPER_ACTION_NAME,
        open_position: float = DEFAULT_OPEN_POSITION,
        closed_position: float = DEFAULT_CLOSED_POSITION,
        server_wait_seconds: float = 5.0,
        result_timeout_padding: float = 5.0,
    ) -> None:
        """Create a client without blocking for controller discovery."""
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        if not isinstance(action_name, str) or not action_name.strip():
            raise ValueError('action_name must not be empty')

        self._node = node
        self._logger = node.get_logger()
        self._action_type = FollowJointTrajectory
        self._open_position = _validate_position(open_position)
        self._closed_position = _validate_position(closed_position)
        self._server_wait_seconds = _validate_non_negative_number(
            'server_wait_seconds', server_wait_seconds
        )
        self._result_timeout_padding = _validate_non_negative_number(
            'result_timeout_padding', result_timeout_padding
        )
        self._action_name = action_name.strip()
        self._action_client = ActionClient(
            node, FollowJointTrajectory, self._action_name
        )

    def open(self, duration: float = DEFAULT_DURATION) -> None:  # noqa: A003
        """Open the gripper over the requested number of seconds."""
        self.move_to(self._open_position, duration)

    def close(self, duration: float = DEFAULT_DURATION) -> None:
        """Close the gripper over the requested number of seconds."""
        self.move_to(self._closed_position, duration)

    def move_to(
        self, position: float, duration: float = DEFAULT_DURATION
    ) -> None:
        """Move the master knuckle joint to an absolute radian position."""
        from action_msgs.msg import GoalStatus

        target = _validate_position(position)
        travel_seconds = _validate_positive_number('duration', duration)

        if not self._action_client.wait_for_server(
            timeout_sec=self._server_wait_seconds
        ):
            raise GripperServerUnavailableError(
                f'Gripper action server {self._action_name} was not '
                f'available within {self._server_wait_seconds:.1f} seconds'
            )

        goal = self._make_goal(target, travel_seconds)
        self._logger.info(
            f'Commanding gripper position {target:.3f} rad over '
            f'{travel_seconds:.2f} seconds'
        )
        send_future = self._action_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, self._server_wait_seconds):
            raise GripperCommandTimeoutError(
                'Timed out while sending the gripper command'
            )

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise GripperCommandRejectedError(
                'The gripper controller rejected the command'
            )

        result_future = goal_handle.get_result_async()
        result_timeout = travel_seconds + self._result_timeout_padding
        if not self._wait_for_future(result_future, result_timeout):
            goal_handle.cancel_goal_async()
            raise GripperCommandTimeoutError(
                'Gripper execution did not finish within '
                f'{result_timeout:.1f} seconds'
            )

        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise GripperCommandFailedError(
                'The gripper controller returned no result'
            )

        controller_result = wrapped_result.result
        successful = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and controller_result.error_code
            == self._action_type.Result.SUCCESSFUL
        )
        if not successful:
            error_text = controller_result.error_string or 'no error text'
            raise GripperCommandFailedError(
                'Gripper command failed: '
                f'goal_status={wrapped_result.status}, '
                f'error_code={controller_result.error_code}, '
                f'message={error_text}'
            )

        self._logger.info(
            f'Gripper reached {target:.3f} rad successfully'
        )

    def destroy(self) -> None:
        """Release the underlying ROS action client."""
        self._action_client.destroy()

    def _make_goal(self, position: float, duration: float) -> Any:
        from rclpy.duration import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal = self._action_type.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]
        return goal

    def _wait_for_future(self, future: Any, timeout_seconds: float) -> bool:
        import rclpy

        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_seconds
        )
        return future.done()


def _validate_position(value: Any) -> float:
    position = _validate_finite_number('position', value)
    if not MIN_POSITION <= position <= MAX_POSITION:
        raise ValueError(
            f'position must be between {MIN_POSITION} and {MAX_POSITION} rad'
        )
    return position


def _validate_positive_number(name: str, value: Any) -> float:
    number = _validate_finite_number(name, value)
    if number <= 0.0:
        raise ValueError(f'{name} must be greater than zero')
    return number


def _validate_non_negative_number(name: str, value: Any) -> float:
    number = _validate_finite_number(name, value)
    if number < 0.0:
        raise ValueError(f'{name} must not be negative')
    return number


def _validate_finite_number(name: str, value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number
