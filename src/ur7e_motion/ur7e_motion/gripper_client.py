"""Robotiq 2F-85 的同步 Python 控制客户端。

夹爪控制器提供的是 FollowJointTrajectory Action。本类把异步 Action 通信包装成
open()/close()/move_to() 这类会等待结果的顺序式调用，便于直接写抓取任务流程。
"""

import math
from typing import Any


# MoveIt/任务节点向这个 Action 发送“主动 knuckle 在多少秒内转到多少弧度”的轨迹。
GRIPPER_ACTION_NAME = '/gripper_controller/follow_joint_trajectory'
# 2F-85 的其余活动关节由 URDF mimic 关系联动，只有这个关节需要直接命令。
GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'
# 当前模型的约定：0 rad 为张开，0.8 rad 为完全闭合。
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
    """向 2F-85 控制器发送带时间参数、会阻塞等待结果的命令。

    ROS 节点的创建和销毁仍由调用方负责；本类只在等待 Action 响应时 spin 该节点。
    因此抓取任务可以自然地写成“移动 → 闭合 → 抬升”的顺序代码。
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
        """创建 Action 客户端；不在构造阶段等待控制器出现。"""
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        if not isinstance(action_name, str) or not action_name.strip():
            raise ValueError('action_name must not be empty')

        self._node = node
        self._logger = node.get_logger()
        # 保存 Action 类型，后续用于构造目标和判断控制器的 SUCCESSFUL 返回码。
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
        """在指定秒数内张开夹爪（主动关节到 0 rad）。"""
        self.move_to(self._open_position, duration)

    def close(self, duration: float = DEFAULT_DURATION) -> None:
        """在指定秒数内完全闭合夹爪（主动关节到 0.8 rad）。"""
        self.move_to(self._closed_position, duration)

    def move_to(
        self, position: float, duration: float = DEFAULT_DURATION
    ) -> None:
        """将主动 knuckle 移到绝对弧度位置，并等待控制器完成。"""
        from action_msgs.msg import GoalStatus

        target = _validate_position(position)
        travel_seconds = _validate_positive_number('duration', duration)

        # 先确认控制器已经提供 Action 服务；否则不能假定夹爪已经收到命令。
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
        # Action 通信有两阶段：先等待“目标是否被接受”，再等待“轨迹是否执行完成”。
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
        # 执行最长允许时间 = 期望轨迹时长 + 网络/控制器响应缓冲时间。
        result_timeout = travel_seconds + self._result_timeout_padding
        if not self._wait_for_future(result_future, result_timeout):
            # 超时后请求取消，避免旧夹爪命令在后续任务步骤中继续执行。
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
        # ROS Action 状态成功还不够；还要确认轨迹控制器自己的 error_code 成功。
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
        """释放内部 ROS Action 客户端。"""
        self._action_client.destroy()

    def _make_goal(self, position: float, duration: float) -> Any:
        """构造只包含一个主动关节、一个终点的 FollowJointTrajectory 目标。"""
        from rclpy.duration import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal = self._action_type.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [position]
        # 控制器据此插值生成从当前角度到目标角度的开合动画。
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]
        return goal

    def _wait_for_future(self, future: Any, timeout_seconds: float) -> bool:
        """在指定超时内 spin 外部节点，直到异步 ROS Future 完成。"""
        import rclpy

        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_seconds
        )
        return future.done()


def _validate_position(value: Any) -> float:
    """验证主动 knuckle 目标位于模型允许的 [0.0, 0.8] rad 范围。"""
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
