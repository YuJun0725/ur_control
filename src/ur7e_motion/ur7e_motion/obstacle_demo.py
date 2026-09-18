"""Run a UR7e motion demo with a table and a path-blocking obstacle."""

from dataclasses import dataclass
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
    PlanningSceneError,
    RobotStateUnavailableError,
    SceneManager,
    UR7eMoveItClient,
)
from ur7e_motion.fixed_task import READY
from ur7e_motion.motion_command import ExitCode
from ur7e_motion.validation import validate_numeric_vector


TABLE_ID = "demo_table"
OBSTACLE_ID = "demo_obstacle"

_MOVEIT_CLIENT: UR7eMoveItClient | None = None


@dataclass(frozen=True)
class ObstacleDemoConfig:
    """Validated geometry and timing parameters for the obstacle demo."""

    table_size: list[float]
    table_position: list[float]
    obstacle_size: list[float]
    obstacle_offset: list[float]
    target_translation: list[float]
    scene_wait_seconds: float
    goal_hold_seconds: float


def execute_obstacle_demo(
    arm: Any,
    scene: Any,
    config: ObstacleDemoConfig,
    *,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Move around a generated obstacle and return to READY."""
    arm.move_to_joint(READY)
    tool_pose = arm.get_current_pose().pose
    obstacle_position = [
        tool_pose.position.x + config.obstacle_offset[0],
        tool_pose.position.y + config.obstacle_offset[1],
        tool_pose.position.z + config.obstacle_offset[2],
    ]

    try:
        scene.add_box(TABLE_ID, config.table_size, config.table_position)
        scene.add_box(OBSTACLE_ID, config.obstacle_size, obstacle_position)
        _sleep_if_positive(config.scene_wait_seconds, sleep_fn)

        arm.move_by_translation(config.target_translation)
        _sleep_if_positive(config.goal_hold_seconds, sleep_fn)
        arm.move_to_joint(READY)
    finally:
        scene.clear()


def _sleep_if_positive(
    duration: float, sleep_fn: Callable[[float], None]
) -> None:
    if duration > 0.0:
        sleep_fn(duration)


def _read_config(node: Any) -> ObstacleDemoConfig:
    defaults = {
        "table_size": [1.4, 1.4, 0.08],
        "table_position": [0.0, 0.0, -0.05],
        "obstacle_size": [0.04, 0.08, 0.08],
        "obstacle_offset": [0.10, 0.0, 0.0],
        "target_translation": [0.20, 0.0, 0.0],
        "scene_wait_seconds": 1.0,
        "goal_hold_seconds": 2.0,
    }
    for name, default in defaults.items():
        node.declare_parameter(name, default)

    table_size = _read_positive_vector(node, "table_size")
    obstacle_size = _read_positive_vector(node, "obstacle_size")
    table_position = _read_vector(node, "table_position")
    obstacle_offset = _read_vector(node, "obstacle_offset")
    target_translation = _read_vector(node, "target_translation")
    scene_wait_seconds = _read_non_negative_number(node, "scene_wait_seconds")
    goal_hold_seconds = _read_non_negative_number(node, "goal_hold_seconds")
    return ObstacleDemoConfig(
        table_size=table_size,
        table_position=table_position,
        obstacle_size=obstacle_size,
        obstacle_offset=obstacle_offset,
        target_translation=target_translation,
        scene_wait_seconds=scene_wait_seconds,
        goal_hold_seconds=goal_hold_seconds,
    )


def _read_vector(node: Any, name: str) -> list[float]:
    return validate_numeric_vector(name, node.get_parameter(name).value, 3)


def _read_positive_vector(node: Any, name: str) -> list[float]:
    values = _read_vector(node, name)
    if any(value <= 0.0 for value in values):
        raise ValueError(f"{name} values must all be greater than zero")
    return values


def _read_non_negative_number(node: Any, name: str) -> float:
    value = node.get_parameter(name).value
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a non-negative finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{name} must be a non-negative finite number"
        ) from exc
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be a non-negative finite number")
    return number


def _run(node: Any, config: ObstacleDemoConfig) -> ExitCode:
    global _MOVEIT_CLIENT

    try:
        _MOVEIT_CLIENT = UR7eMoveItClient(node)
        scene = SceneManager(node, _MOVEIT_CLIENT.planning_scene_monitor)
        node.get_logger().info(
            "Starting obstacle demo: READY -> add scene -> avoid obstacle "
            "-> READY"
        )
        execute_obstacle_demo(_MOVEIT_CLIENT, scene, config)
        node.get_logger().info("Obstacle avoidance demo completed")
        return ExitCode.SUCCESS
    except MoveItDependencyError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.DEPENDENCY_ERROR
    except RobotStateUnavailableError as exc:
        node.get_logger().error(f"Obstacle demo stopped: {exc}")
        return ExitCode.ROBOT_STATE_UNAVAILABLE
    except MotionPlanningError as exc:
        node.get_logger().error(f"Obstacle demo stopped: {exc}")
        return ExitCode.PLANNING_FAILED
    except MotionExecutionError as exc:
        node.get_logger().error(f"Obstacle demo stopped: {exc}")
        return ExitCode.EXECUTION_FAILED
    except PlanningSceneError as exc:
        node.get_logger().error(f"Obstacle demo stopped: {exc}")
        return ExitCode.SCENE_FAILED
    except Exception as exc:  # MoveIt bindings can raise C++ exception types.
        node.get_logger().error(
            f"Unexpected obstacle demo error: {type(exc).__name__}: {exc}"
        )
        return ExitCode.INTERNAL_ERROR


def main(args: list[str] | None = None) -> int:
    """ROS console entry point for the obstacle avoidance demo."""
    rclpy.init(args=args)
    node = rclpy.create_node("ur7e_obstacle_demo_parameters")
    exit_code = ExitCode.INTERNAL_ERROR
    try:
        try:
            config = _read_config(node)
        except (TypeError, ValueError) as exc:
            node.get_logger().error(f"Invalid obstacle demo parameter: {exc}")
            exit_code = ExitCode.INVALID_PARAMETERS
        else:
            exit_code = _run(node, config)
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
