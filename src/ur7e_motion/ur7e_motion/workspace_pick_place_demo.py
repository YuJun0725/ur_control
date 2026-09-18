"""Run a collision-aware pick-and-place demo in a fixed workcell scene."""

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
from ur7e_motion.pick_place_demo import GRIPPER_TOUCH_LINKS, TCP_LINK
from ur7e_motion.validation import validate_numeric_vector


TABLE_ID = 'workspace_table'
BACK_WALL_ID = 'workspace_back_wall'
STORAGE_BOX_ID = 'workspace_storage_box'
CENTER_DIVIDER_ID = 'workspace_center_divider'
TARGET_ID = 'workspace_target'

TABLE_COLOR = (0.45, 0.25, 0.10, 1.0)
BACK_WALL_COLOR = (0.60, 0.66, 0.75, 1.0)
STORAGE_BOX_COLOR = (0.10, 0.38, 0.82, 1.0)
CENTER_DIVIDER_COLOR = (0.95, 0.48, 0.08, 1.0)
TARGET_COLOR = (0.88, 0.08, 0.08, 1.0)

_MOVEIT_CLIENT: UR7eMoveItClient | None = None
_GRIPPER_CLIENT: GripperClient | None = None


@dataclass(frozen=True)
class WorkspacePickPlaceConfig:
    """Validated fixed-scene geometry and task parameters."""

    table_size: list[float]
    table_position: list[float]
    back_wall_size: list[float]
    back_wall_position: list[float]
    storage_box_size: list[float]
    storage_box_position: list[float]
    center_divider_size: list[float]
    center_divider_position: list[float]
    target_size: list[float]
    target_position: list[float]
    place_position: list[float]
    grasp_tcp_offset: list[float]
    lift_translation: list[float]
    grasp_position: float
    gripper_duration: float
    scene_wait_seconds: float
    static_scene_before_ready: bool = False


def execute_workspace_pick_place(
    arm: Any,
    gripper: Any,
    scene: Any,
    config: WorkspacePickPlaceConfig,
    *,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Pick one box in the workcell, place it, and leave the scene visible."""
    attached = False
    touch_allowed = False
    target_added = False

    try:
        if config.static_scene_before_ready:
            _add_static_workcell(scene, config)
        arm.move_to_joint(READY)
        gripper.open(config.gripper_duration)
        ready_pose = arm.get_current_pose().pose
        orientation = [
            ready_pose.orientation.x,
            ready_pose.orientation.y,
            ready_pose.orientation.z,
            ready_pose.orientation.w,
        ]

        if not config.static_scene_before_ready:
            _add_static_workcell(scene, config)
        scene.add_box(
            TARGET_ID,
            config.target_size,
            config.target_position,
            color=TARGET_COLOR,
        )
        target_added = True
        _sleep_if_positive(config.scene_wait_seconds, sleep_fn)

        scene.set_object_touch_allowed(
            TARGET_ID, GRIPPER_TOUCH_LINKS, True
        )
        touch_allowed = True

        source_grasp_tcp = _add_vectors(
            config.target_position, config.grasp_tcp_offset
        )
        source_pregrasp_tcp = _add_vectors(
            source_grasp_tcp, config.lift_translation
        )
        place_grasp_tcp = _add_vectors(
            config.place_position, config.grasp_tcp_offset
        )
        place_pregrasp_tcp = _add_vectors(
            place_grasp_tcp, config.lift_translation
        )

        arm.move_to_pose(source_pregrasp_tcp, orientation)
        arm.move_to_pose(source_grasp_tcp, orientation)
        gripper.move_to(config.grasp_position, config.gripper_duration)
        scene.attach_object(
            TARGET_ID,
            link_name=TCP_LINK,
            touch_links=GRIPPER_TOUCH_LINKS,
        )
        attached = True

        arm.move_to_pose(source_pregrasp_tcp, orientation)
        arm.move_to_pose(place_pregrasp_tcp, orientation)
        arm.move_to_pose(place_grasp_tcp, orientation)
        gripper.open(config.gripper_duration)
        scene.detach_object(TARGET_ID)
        attached = False
        arm.move_to_pose(place_pregrasp_tcp, orientation)
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
        if target_added:
            _ignore_cleanup_error(scene.remove_object, TARGET_ID)
        raise


def _add_static_workcell(scene: Any, config: WorkspacePickPlaceConfig) -> None:
    """Add or replace the persistent fixed collision objects."""
    for object_id, size, position, color in (
        (TABLE_ID, config.table_size, config.table_position, TABLE_COLOR),
        (
            BACK_WALL_ID,
            config.back_wall_size,
            config.back_wall_position,
            BACK_WALL_COLOR,
        ),
        (
            STORAGE_BOX_ID,
            config.storage_box_size,
            config.storage_box_position,
            STORAGE_BOX_COLOR,
        ),
        (
            CENTER_DIVIDER_ID,
            config.center_divider_size,
            config.center_divider_position,
            CENTER_DIVIDER_COLOR,
        ),
    ):
        scene.add_box(object_id, size, position, color=color)


def _add_vectors(first: list[float], second: list[float]) -> list[float]:
    """Add coordinate vectors without exposing binary float display noise."""
    return [round(first[index] + second[index], 10) for index in range(3)]


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


def _read_config(node: Any) -> WorkspacePickPlaceConfig:
    defaults = {
        'table_size': [0.70, 0.45, 0.30],
        'table_position': [0.0, 0.48, 0.15],
        'back_wall_size': [0.70, 0.05, 0.55],
        'back_wall_position': [0.0, 0.75, 0.575],
        'storage_box_size': [0.14, 0.14, 0.20],
        'storage_box_position': [-0.26, 0.62, 0.40],
        'center_divider_size': [0.06, 0.18, 0.24],
        'center_divider_position': [0.02, 0.47, 0.42],
        'target_size': [0.035, 0.035, 0.060],
        'target_position': [0.16, 0.47, 0.34],
        'place_position': [0.16, 0.32, 0.34],
        'grasp_tcp_offset': [0.0, 0.0, -0.03],
        'lift_translation': [0.0, 0.0, 0.10],
        'grasp_position': 0.50,
        'gripper_duration': 1.0,
        'scene_wait_seconds': 0.5,
        'static_scene_before_ready': False,
    }
    for name, default in defaults.items():
        node.declare_parameter(name, default)

    vector_names = (
        'table_size',
        'table_position',
        'back_wall_size',
        'back_wall_position',
        'storage_box_size',
        'storage_box_position',
        'center_divider_size',
        'center_divider_position',
        'target_size',
        'target_position',
        'place_position',
        'grasp_tcp_offset',
        'lift_translation',
    )
    values = {name: _read_vector(node, name) for name in vector_names}
    for name in (
        'table_size',
        'back_wall_size',
        'storage_box_size',
        'center_divider_size',
        'target_size',
    ):
        if any(value <= 0.0 for value in values[name]):
            raise ValueError(f'{name} values must all be greater than zero')

    grasp_position = _read_finite_number(node, 'grasp_position')
    if not 0.0 <= grasp_position <= 0.8:
        raise ValueError('grasp_position must be between 0.0 and 0.8 rad')

    gripper_duration = _read_finite_number(node, 'gripper_duration')
    if gripper_duration <= 0.0:
        raise ValueError('gripper_duration must be greater than zero')

    scene_wait_seconds = _read_finite_number(node, 'scene_wait_seconds')
    if scene_wait_seconds < 0.0:
        raise ValueError('scene_wait_seconds must not be negative')

    static_scene_before_ready = node.get_parameter(
        'static_scene_before_ready'
    ).value
    if not isinstance(static_scene_before_ready, bool):
        raise ValueError('static_scene_before_ready must be a bool')

    lift_translation = values['lift_translation']
    if lift_translation[2] <= 0.0:
        raise ValueError('lift_translation must have a positive Z component')

    return WorkspacePickPlaceConfig(
        table_size=values['table_size'],
        table_position=values['table_position'],
        back_wall_size=values['back_wall_size'],
        back_wall_position=values['back_wall_position'],
        storage_box_size=values['storage_box_size'],
        storage_box_position=values['storage_box_position'],
        center_divider_size=values['center_divider_size'],
        center_divider_position=values['center_divider_position'],
        target_size=values['target_size'],
        target_position=values['target_position'],
        place_position=values['place_position'],
        grasp_tcp_offset=values['grasp_tcp_offset'],
        lift_translation=lift_translation,
        grasp_position=grasp_position,
        gripper_duration=gripper_duration,
        scene_wait_seconds=scene_wait_seconds,
        static_scene_before_ready=static_scene_before_ready,
    )


def _read_vector(node: Any, name: str) -> list[float]:
    return validate_numeric_vector(name, node.get_parameter(name).value, 3)


def _read_finite_number(node: Any, name: str) -> float:
    value = node.get_parameter(name).value
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def _run(node: Any, config: WorkspacePickPlaceConfig) -> ExitCode:
    global _GRIPPER_CLIENT, _MOVEIT_CLIENT

    try:
        _MOVEIT_CLIENT = UR7eMoveItClient(node)
        _GRIPPER_CLIENT = GripperClient(node)
        scene = SceneManager(node, _MOVEIT_CLIENT.planning_scene_monitor)
        node.get_logger().info(
            'Starting workspace pick-place demo: READY -> avoid divider '
            '-> PICK -> PLACE -> READY'
        )
        execute_workspace_pick_place(_MOVEIT_CLIENT, _GRIPPER_CLIENT, scene, config)
        node.get_logger().info(
            'Workspace pick-place demo completed; scene remains in RViz'
        )
        return ExitCode.SUCCESS
    except MoveItDependencyError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.DEPENDENCY_ERROR
    except RobotStateUnavailableError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.ROBOT_STATE_UNAVAILABLE
    except MotionPlanningError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.PLANNING_FAILED
    except MotionExecutionError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.EXECUTION_FAILED
    except PlanningSceneError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.SCENE_FAILED
    except GripperControlError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.GRIPPER_FAILED
    except Exception as exc:
        node.get_logger().error(
            f'Unexpected workspace demo error: {type(exc).__name__}: {exc}'
        )
        return ExitCode.INTERNAL_ERROR


def main(args: list[str] | None = None) -> int:
    """ROS console entry point for the persistent workcell demonstration."""
    rclpy.init(args=args)
    node = rclpy.create_node('ur7e_workspace_pick_place_parameters')
    exit_code = ExitCode.INTERNAL_ERROR
    try:
        try:
            config = _read_config(node)
        except (TypeError, ValueError) as exc:
            node.get_logger().error(f'Invalid workspace demo parameter: {exc}')
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


if __name__ == '__main__':
    raise SystemExit(main())
