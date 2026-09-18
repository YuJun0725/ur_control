"""Plan and execute one UR7e motion, then exit."""

from enum import IntEnum
import os
import sys
from typing import Any

import rclpy

from ur7e_motion.moveit_client import (
    MotionExecutionError,
    MotionPlanningError,
    MoveItDependencyError,
    RobotStateUnavailableError,
    UR7eMoveItClient,
)
from ur7e_motion.validation import validate_mode, validate_numeric_vector


DEFAULT_JOINT_POSITIONS = [
    1.54,
    -1.62,
    1.4,
    -1.350796327,
    -1.570796326589793,
    -0.030796327,
]
DEFAULT_TRANSLATION = [0.0, 0.0, 0.05]

# MoveIt 2.12.4 can segfault while destroying MoveItPy during interpreter
# teardown. Keep the client alive and let the OS release it on process exit.
_MOVEIT_CLIENT: UR7eMoveItClient | None = None


class ExitCode(IntEnum):
    """Process exit codes for automation and diagnosis."""

    SUCCESS = 0
    INVALID_PARAMETERS = 2
    DEPENDENCY_ERROR = 3
    ROBOT_STATE_UNAVAILABLE = 4
    PLANNING_FAILED = 5
    EXECUTION_FAILED = 6
    SCENE_FAILED = 7
    GRIPPER_FAILED = 8
    INTERNAL_ERROR = 10


def _declare_parameters(node: Any) -> tuple[str, list[float], list[float]]:
    node.declare_parameter("mode", "joint")
    node.declare_parameter("joint_positions", DEFAULT_JOINT_POSITIONS)
    node.declare_parameter("translation", DEFAULT_TRANSLATION)

    mode = validate_mode(node.get_parameter("mode").value)
    joint_positions = validate_numeric_vector(
        "joint_positions", node.get_parameter("joint_positions").value, 6
    )
    translation = validate_numeric_vector(
        "translation", node.get_parameter("translation").value, 3
    )
    return mode, joint_positions, translation


def _run_motion(
    node: Any,
    mode: str,
    joint_positions: list[float],
    translation: list[float],
) -> ExitCode:
    global _MOVEIT_CLIENT

    try:
        _MOVEIT_CLIENT = UR7eMoveItClient(node)
        if mode == "joint":
            _MOVEIT_CLIENT.move_to_joint(joint_positions)
        else:
            _MOVEIT_CLIENT.move_by_translation(translation)
        return ExitCode.SUCCESS
    except MoveItDependencyError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.DEPENDENCY_ERROR
    except RobotStateUnavailableError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.ROBOT_STATE_UNAVAILABLE
    except MotionPlanningError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.PLANNING_FAILED
    except MotionExecutionError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.EXECUTION_FAILED
    except Exception as exc:  # MoveIt bindings can raise C++ exception types.
        node.get_logger().error(
            f"Unexpected motion error: {type(exc).__name__}: {exc}"
        )
        return ExitCode.INTERNAL_ERROR


def main(args: list[str] | None = None) -> int:
    """ROS console entry point."""
    rclpy.init(args=args)
    node = rclpy.create_node("ur7e_motion_parameters")
    exit_code = ExitCode.INTERNAL_ERROR
    try:
        try:
            mode, joint_positions, translation = _declare_parameters(node)
        except (TypeError, ValueError) as exc:
            node.get_logger().error(f"Invalid motion parameters: {exc}")
            exit_code = ExitCode.INVALID_PARAMETERS
        else:
            exit_code = _run_motion(node, mode, joint_positions, translation)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if _MOVEIT_CLIENT is not None:
        # Avoid the known MoveItPy 2.12.4 destructor crash while preserving an
        # accurate process exit code for launch and automation.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(int(exit_code))
    return int(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
