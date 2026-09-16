"""Run a fixed, multi-step UR7e demonstration task."""

import math
import os
import sys
import time
from typing import Any, Callable

import rclpy

from ur7e_motion import (
    MotionExecutionError,
    MotionPlanningError,
    MoveItDependencyError,
    RobotStateUnavailableError,
    UR7eMoveItClient,
)
from ur7e_motion.motion_command import ExitCode


# Modify these constants and ``execute_task`` to define another fixed job.
READY = [1.54, -1.62, 1.4, -1.2, -1.6, -0.11]
MOVE_UP = [0.0, 0.0, 0.05]
MOVE_X_POSITIVE = [0.25, 0.0, 0.0]
MOVE_Y_POSITIVE = [0.0, 0.15, 0.0]
MOVE_X_NEGATIVE = [-0.25, 0.0, 0.0]
MOVE_Y_NEGATIVE = [0.0, -0.25, 0.0]
MOVE_DOWN = [0.0, 0.0, -0.15]

# Keep MoveItPy alive until the process exits. MoveIt 2.12.4 can crash while
# destroying this object during Python interpreter teardown.
_MOVEIT_CLIENT: UR7eMoveItClient | None = None


def execute_task(
    arm: Any,
    pause_seconds: float = 1.0,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Move to READY, trace a raised 5 cm square, then return to READY.

    An exception from any motion is deliberately allowed to propagate so that
    none of the remaining steps are executed after a failure.
    """
    arm.move_to_joint(READY)

    translations = (
        MOVE_UP,
        MOVE_X_POSITIVE,
        MOVE_Y_POSITIVE,
        MOVE_X_NEGATIVE,
        MOVE_Y_NEGATIVE,
        MOVE_DOWN,
    )
    for translation in translations:
        arm.move_by_translation(translation)
        if pause_seconds > 0.0:
            sleep_fn(pause_seconds)

    arm.move_to_joint(READY)


def _read_pause_seconds(node: Any) -> float:
    node.declare_parameter("pause_seconds", 1.0)
    value = node.get_parameter("pause_seconds").value
    if isinstance(value, bool):
        raise ValueError("pause_seconds must be a non-negative finite number")
    try:
        pause_seconds = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "pause_seconds must be a non-negative finite number"
        ) from exc
    if not math.isfinite(pause_seconds) or pause_seconds < 0.0:
        raise ValueError("pause_seconds must be a non-negative finite number")
    return pause_seconds


def _run(node: Any, pause_seconds: float) -> ExitCode:
    global _MOVEIT_CLIENT

    try:
        _MOVEIT_CLIENT = UR7eMoveItClient(node)
        node.get_logger().info(
            "Starting fixed task: READY -> UP -> XY SQUARE -> DOWN -> READY"
        )
        execute_task(_MOVEIT_CLIENT, pause_seconds)
        node.get_logger().info("Fixed task completed")
        return ExitCode.SUCCESS
    except MoveItDependencyError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.DEPENDENCY_ERROR
    except RobotStateUnavailableError as exc:
        node.get_logger().error(f"Fixed task stopped: {exc}")
        return ExitCode.ROBOT_STATE_UNAVAILABLE
    except MotionPlanningError as exc:
        node.get_logger().error(f"Fixed task stopped: {exc}")
        return ExitCode.PLANNING_FAILED
    except MotionExecutionError as exc:
        node.get_logger().error(f"Fixed task stopped: {exc}")
        return ExitCode.EXECUTION_FAILED
    except Exception as exc:  # MoveIt bindings can raise C++ exception types.
        node.get_logger().error(
            f"Unexpected fixed task error: {type(exc).__name__}: {exc}"
        )
        return ExitCode.INTERNAL_ERROR


def main(args: list[str] | None = None) -> int:
    """ROS console entry point for the fixed task."""
    rclpy.init(args=args)
    node = rclpy.create_node("ur7e_fixed_task_parameters")
    exit_code = ExitCode.INTERNAL_ERROR
    try:
        try:
            pause_seconds = _read_pause_seconds(node)
        except ValueError as exc:
            node.get_logger().error(f"Invalid fixed task parameter: {exc}")
            exit_code = ExitCode.INVALID_PARAMETERS
        else:
            exit_code = _run(node, pause_seconds)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if _MOVEIT_CLIENT is not None:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(int(exit_code))
    return int(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
