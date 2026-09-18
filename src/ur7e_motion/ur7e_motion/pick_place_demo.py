"""Run a logical RViz pick-and-place demonstration with the Robotiq gripper."""

from dataclasses import dataclass
import math
import os
import sys
import time
from typing import Any, Callable

import rclpy

from ur7e_motion import (
    GripperClient,
    GripperControlError,
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


TARGET_ID = "pick_place_target"
TCP_LINK = "robotiq_tcp"
GRIPPER_TOUCH_LINKS = (
    "robotiq_85_left_knuckle_link",
    "robotiq_85_left_finger_link",
    "robotiq_85_left_inner_knuckle_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_knuckle_link",
    "robotiq_85_right_finger_link",
    "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_right_finger_tip_link",
)

_MOVEIT_CLIENT: UR7eMoveItClient | None = None
_GRIPPER_CLIENT: GripperClient | None = None


@dataclass(frozen=True)
class PickPlaceConfig:
    """Validated geometry, motion and timing parameters for the demo."""

    object_size: list[float]
    object_offset_from_ready_tcp: list[float]
    approach_translation: list[float]
    lift_translation: list[float]
    transport_translation: list[float]
    place_translation: list[float]
    retreat_translation: list[float]
    grasp_position: float
    gripper_duration: float
    scene_wait_seconds: float


def execute_pick_place(
    arm: Any,
    gripper: Any,
    scene: Any,
    config: PickPlaceConfig,
    *,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Execute one pick-and-place sequence and leave the box at its goal."""
    attached = False
    touch_allowed = False
    object_added = False

    try:
        arm.move_to_joint(READY)
        gripper.open(config.gripper_duration)

        ready_pose = arm.get_current_pose().pose
        object_position = [
            ready_pose.position.x + config.object_offset_from_ready_tcp[0],
            ready_pose.position.y + config.object_offset_from_ready_tcp[1],
            ready_pose.position.z + config.object_offset_from_ready_tcp[2],
        ]
        scene.add_box(TARGET_ID, config.object_size, object_position)
        object_added = True
        _sleep_if_positive(config.scene_wait_seconds, sleep_fn)

        scene.set_object_touch_allowed(
            TARGET_ID, GRIPPER_TOUCH_LINKS, True
        )
        touch_allowed = True
        arm.move_by_translation(config.approach_translation)
        gripper.move_to(
            config.grasp_position, config.gripper_duration
        )
        scene.attach_object(
            TARGET_ID,
            link_name=TCP_LINK,
            touch_links=GRIPPER_TOUCH_LINKS,
        )
        attached = True

        arm.move_by_translation(config.lift_translation)
        arm.move_by_translation(config.transport_translation)
        arm.move_by_translation(config.place_translation)

        gripper.open(config.gripper_duration)
        scene.detach_object(TARGET_ID)
        attached = False
        arm.move_by_translation(config.retreat_translation)
        scene.set_object_touch_allowed(
            TARGET_ID, GRIPPER_TOUCH_LINKS, False
        )
        touch_allowed = False
        arm.move_to_joint(READY)
    except Exception:
        if attached:
            _ignore_cleanup_error(scene.detach_object, TARGET_ID)
        if touch_allowed:
            _ignore_cleanup_error(
                scene.set_object_touch_allowed,
                TARGET_ID,
                GRIPPER_TOUCH_LINKS,
                False,
            )
        if object_added:
            _ignore_cleanup_error(scene.remove_object, TARGET_ID)
        raise


def _ignore_cleanup_error(function: Callable[..., Any], *args: Any) -> None:
    try:
        function(*args)
    except Exception:
        pass


def _sleep_if_positive(
    duration: float, sleep_fn: Callable[[float], None]
) -> None:
    if duration > 0.0:
        sleep_fn(duration)


def _read_config(node: Any) -> PickPlaceConfig:
    defaults = {
        "object_size": [0.035, 0.035, 0.060],
        "object_offset_from_ready_tcp": [0.0, 0.0, -0.07],
        "approach_translation": [0.0, 0.0, -0.10],
        "lift_translation": [0.0, 0.0, 0.10],
        "transport_translation": [0.15, 0.0, 0.0],
        "place_translation": [0.0, 0.0, -0.10],
        "retreat_translation": [0.0, 0.0, 0.10],
        "grasp_position": 0.50,
        "gripper_duration": 1.0,
        "scene_wait_seconds": 0.5,
    }
    for name, default in defaults.items():
        node.declare_parameter(name, default)

    object_size = _read_vector(node, "object_size")
    if any(value <= 0.0 for value in object_size):
        raise ValueError("object_size values must all be greater than zero")

    grasp_position = _read_finite_number(node, "grasp_position")
    if not 0.0 <= grasp_position <= 0.8:
        raise ValueError("grasp_position must be between 0.0 and 0.8 rad")

    gripper_duration = _read_finite_number(node, "gripper_duration")
    if gripper_duration <= 0.0:
        raise ValueError("gripper_duration must be greater than zero")

    scene_wait_seconds = _read_finite_number(node, "scene_wait_seconds")
    if scene_wait_seconds < 0.0:
        raise ValueError("scene_wait_seconds must not be negative")

    return PickPlaceConfig(
        object_size=object_size,
        object_offset_from_ready_tcp=_read_vector(
            node, "object_offset_from_ready_tcp"
        ),
        approach_translation=_read_vector(node, "approach_translation"),
        lift_translation=_read_vector(node, "lift_translation"),
        transport_translation=_read_vector(node, "transport_translation"),
        place_translation=_read_vector(node, "place_translation"),
        retreat_translation=_read_vector(node, "retreat_translation"),
        grasp_position=grasp_position,
        gripper_duration=gripper_duration,
        scene_wait_seconds=scene_wait_seconds,
    )


def _read_vector(node: Any, name: str) -> list[float]:
    return validate_numeric_vector(name, node.get_parameter(name).value, 3)


def _read_finite_number(node: Any, name: str) -> float:
    value = node.get_parameter(name).value
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _run(node: Any, config: PickPlaceConfig) -> ExitCode:
    global _GRIPPER_CLIENT, _MOVEIT_CLIENT

    try:
        _MOVEIT_CLIENT = UR7eMoveItClient(node)
        _GRIPPER_CLIENT = GripperClient(node)
        scene = SceneManager(node, _MOVEIT_CLIENT.planning_scene_monitor)
        node.get_logger().info(
            "Starting pick-place demo: READY -> PICK -> TRANSFER -> PLACE "
            "-> READY"
        )
        execute_pick_place(
            _MOVEIT_CLIENT, _GRIPPER_CLIENT, scene, config
        )
        node.get_logger().info("Pick-place demo completed")
        return ExitCode.SUCCESS
    except MoveItDependencyError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.DEPENDENCY_ERROR
    except RobotStateUnavailableError as exc:
        node.get_logger().error(f"Pick-place demo stopped: {exc}")
        return ExitCode.ROBOT_STATE_UNAVAILABLE
    except MotionPlanningError as exc:
        node.get_logger().error(f"Pick-place demo stopped: {exc}")
        return ExitCode.PLANNING_FAILED
    except MotionExecutionError as exc:
        node.get_logger().error(f"Pick-place demo stopped: {exc}")
        return ExitCode.EXECUTION_FAILED
    except PlanningSceneError as exc:
        node.get_logger().error(f"Pick-place demo stopped: {exc}")
        return ExitCode.SCENE_FAILED
    except GripperControlError as exc:
        node.get_logger().error(f"Pick-place demo stopped: {exc}")
        return ExitCode.GRIPPER_FAILED
    except Exception as exc:
        node.get_logger().error(
            f"Unexpected pick-place demo error: {type(exc).__name__}: {exc}"
        )
        return ExitCode.INTERNAL_ERROR


def main(args: list[str] | None = None) -> int:
    """ROS console entry point."""
    rclpy.init(args=args)
    node = rclpy.create_node("ur7e_pick_place_parameters")
    exit_code = ExitCode.INTERNAL_ERROR
    try:
        try:
            config = _read_config(node)
        except (TypeError, ValueError) as exc:
            node.get_logger().error(f"Invalid pick-place parameter: {exc}")
            exit_code = ExitCode.INVALID_PARAMETERS
        else:
            exit_code = _run(node, config)
    finally:
        if _GRIPPER_CLIENT is not None:
            _GRIPPER_CLIENT.destroy()
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
