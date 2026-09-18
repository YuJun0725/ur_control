"""Tests for the logical RViz pick-and-place task."""

from types import SimpleNamespace

import pytest

from ur7e_motion import MotionPlanningError
from ur7e_motion.fixed_task import READY
from ur7e_motion.pick_place_demo import (
    _read_config,
    GRIPPER_TOUCH_LINKS,
    PickPlaceConfig,
    TARGET_ID,
    TCP_LINK,
    execute_pick_place,
)


def _config():
    return PickPlaceConfig(
        object_size=[0.035, 0.035, 0.060],
        object_offset_from_ready_tcp=[0.0, 0.0, -0.07],
        approach_translation=[0.0, 0.0, -0.10],
        lift_translation=[0.0, 0.0, 0.10],
        transport_translation=[0.15, 0.0, 0.0],
        place_translation=[0.0, 0.0, -0.10],
        retreat_translation=[0.0, 0.0, 0.10],
        grasp_position=0.50,
        gripper_duration=1.0,
        scene_wait_seconds=0.5,
    )


class FakeParameterNode:
    """Provide ROS-like parameter declaration and lookup for validation."""

    def __init__(self, overrides):
        self.values = dict(overrides)

    def declare_parameter(self, name, default):
        self.values.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


class FakeArm:
    """Record arm calls and optionally fail on one translation."""

    def __init__(self, calls, fail_translation=None):
        self.calls = calls
        self.fail_translation = fail_translation
        self.translation_count = 0

    def move_to_joint(self, positions):
        self.calls.append(("joint", positions))

    def get_current_pose(self):
        self.calls.append(("current_pose",))
        return SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.40, y=-0.20, z=0.50)
            )
        )

    def move_by_translation(self, translation):
        self.translation_count += 1
        self.calls.append(("translation", translation))
        if self.translation_count == self.fail_translation:
            raise MotionPlanningError("blocked")


class FakeGripper:
    """Record open and partial-close commands."""

    def __init__(self, calls):
        self.calls = calls

    def open(self, duration):
        self.calls.append(("open", duration))

    def move_to(self, position, duration):
        self.calls.append(("gripper", position, duration))


class FakeScene:
    """Record object ownership and collision-permission changes."""

    def __init__(self, calls):
        self.calls = calls

    def add_box(self, object_id, size, position):
        self.calls.append(("add_box", object_id, size, position))

    def set_object_touch_allowed(self, object_id, links, allowed):
        self.calls.append(("touch", object_id, tuple(links), allowed))

    def attach_object(self, object_id, *, link_name, touch_links):
        self.calls.append(
            ("attach", object_id, link_name, tuple(touch_links))
        )

    def detach_object(self, object_id):
        self.calls.append(("detach", object_id))

    def remove_object(self, object_id):
        self.calls.append(("remove", object_id))


def test_pick_place_executes_complete_sequence():
    """The object is grasped, transferred, released and left in the world."""
    calls = []
    config = _config()

    execute_pick_place(
        FakeArm(calls),
        FakeGripper(calls),
        FakeScene(calls),
        config,
        sleep_fn=lambda value: calls.append(("sleep", value)),
    )

    assert calls == [
        ("joint", READY),
        ("open", 1.0),
        ("current_pose",),
        (
            "add_box",
            TARGET_ID,
            [0.035, 0.035, 0.060],
            [0.40, -0.20, 0.43],
        ),
        ("sleep", 0.5),
        ("touch", TARGET_ID, GRIPPER_TOUCH_LINKS, True),
        ("translation", [0.0, 0.0, -0.10]),
        ("gripper", 0.50, 1.0),
        ("attach", TARGET_ID, TCP_LINK, GRIPPER_TOUCH_LINKS),
        ("translation", [0.0, 0.0, 0.10]),
        ("translation", [0.15, 0.0, 0.0]),
        ("translation", [0.0, 0.0, -0.10]),
        ("open", 1.0),
        ("detach", TARGET_ID),
        ("translation", [0.0, 0.0, 0.10]),
        ("touch", TARGET_ID, GRIPPER_TOUCH_LINKS, False),
        ("joint", READY),
    ]


def test_failure_after_attachment_detaches_and_removes_object():
    """A transfer failure cleans attached state and stops later motions."""
    calls = []

    with pytest.raises(MotionPlanningError, match="blocked"):
        execute_pick_place(
            FakeArm(calls, fail_translation=3),
            FakeGripper(calls),
            FakeScene(calls),
            _config(),
            sleep_fn=lambda value: None,
        )

    assert calls[-3:] == [
        ("detach", TARGET_ID),
        ("touch", TARGET_ID, GRIPPER_TOUCH_LINKS, False),
        ("remove", TARGET_ID),
    ]
    assert calls.count(("joint", READY)) == 1


def test_failure_before_attachment_removes_world_object():
    """An approach failure clears permissions without attempting detach."""
    calls = []

    with pytest.raises(MotionPlanningError, match="blocked"):
        execute_pick_place(
            FakeArm(calls, fail_translation=1),
            FakeGripper(calls),
            FakeScene(calls),
            _config(),
            sleep_fn=lambda value: None,
        )

    assert ("detach", TARGET_ID) not in calls
    assert calls[-2:] == [
        ("touch", TARGET_ID, GRIPPER_TOUCH_LINKS, False),
        ("remove", TARGET_ID),
    ]


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"object_size": [0.03, -0.01, 0.06]}, "object_size"),
        ({"approach_translation": [0.0, -0.1]}, "exactly 3"),
        ({"grasp_position": 0.81}, "grasp_position"),
        ({"gripper_duration": 0.0}, "gripper_duration"),
        ({"scene_wait_seconds": -0.1}, "scene_wait_seconds"),
    ],
)
def test_invalid_pick_place_parameters_are_rejected(overrides, message):
    """Unsafe task parameters are rejected before clients are created."""
    with pytest.raises(ValueError, match=message):
        _read_config(FakeParameterNode(overrides))
