"""Tests for the persistent workcell pick-and-place task."""

from types import SimpleNamespace

import pytest

from ur7e_motion import MotionPlanningError
from ur7e_motion.fixed_task import READY
from ur7e_motion.pick_place_demo import GRIPPER_TOUCH_LINKS
from ur7e_motion.workspace_pick_place_demo import (
    _read_config,
    BACK_WALL_COLOR,
    BACK_WALL_ID,
    CENTER_DIVIDER_COLOR,
    CENTER_DIVIDER_ID,
    execute_workspace_pick_place,
    STORAGE_BOX_COLOR,
    STORAGE_BOX_ID,
    TABLE_COLOR,
    TABLE_ID,
    TARGET_COLOR,
    TARGET_ID,
    WorkspacePickPlaceConfig,
)


def _config() -> WorkspacePickPlaceConfig:
    return WorkspacePickPlaceConfig(
        table_size=[0.70, 0.45, 0.30],
        table_position=[0.0, 0.48, 0.15],
        back_wall_size=[0.70, 0.05, 0.55],
        back_wall_position=[0.0, 0.75, 0.575],
        storage_box_size=[0.14, 0.14, 0.20],
        storage_box_position=[-0.26, 0.62, 0.40],
        center_divider_size=[0.06, 0.18, 0.24],
        center_divider_position=[0.02, 0.47, 0.42],
        target_size=[0.035, 0.035, 0.060],
        target_position=[0.16, 0.47, 0.34],
        place_position=[0.16, 0.32, 0.34],
        grasp_tcp_offset=[0.0, 0.0, -0.03],
        lift_translation=[0.0, 0.0, 0.10],
        grasp_position=0.50,
        gripper_duration=1.0,
        scene_wait_seconds=0.5,
    )


class FakeParameterNode:
    """Provide the parameter subset required by the configuration reader."""

    def __init__(self, overrides):
        self.values = dict(overrides)

    def declare_parameter(self, name, default):
        self.values.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


class FakeArm:
    """Record arm commands and optionally fail one pose planning request."""

    def __init__(self, calls, fail_pose=None):
        self.calls = calls
        self.fail_pose = fail_pose
        self.pose_count = 0

    def move_to_joint(self, positions):
        self.calls.append(('joint', positions))

    def get_current_pose(self):
        self.calls.append(('current_pose',))
        return SimpleNamespace(
            pose=SimpleNamespace(
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            )
        )

    def move_to_pose(self, position, orientation):
        self.pose_count += 1
        self.calls.append(('pose', position, orientation))
        if self.pose_count == self.fail_pose:
            raise MotionPlanningError('blocked')


class FakeGripper:
    """Record gripper commands and optionally reject the partial close."""

    def __init__(self, calls, fail_close=False):
        self.calls = calls
        self.fail_close = fail_close

    def open(self, duration):  # noqa: A003 - matches GripperClient's API.
        self.calls.append(('open', duration))

    def move_to(self, position, duration):
        self.calls.append(('gripper', position, duration))
        if self.fail_close:
            raise RuntimeError('gripper failed')


class FakeScene:
    """Record scene mutations without requiring a MoveIt installation."""

    def __init__(self, calls):
        self.calls = calls

    def add_box(self, object_id, size, position, *, color=None):
        self.calls.append(('add_box', object_id, size, position, color))

    def set_object_touch_allowed(self, object_id, links, allowed):
        self.calls.append(('touch', object_id, tuple(links), allowed))

    def attach_object(self, object_id, *, link_name, touch_links):
        self.calls.append(('attach', object_id, link_name, tuple(touch_links)))

    def detach_object(self, object_id):
        self.calls.append(('detach', object_id))

    def remove_object(self, object_id):
        self.calls.append(('remove', object_id))


def test_workspace_pick_place_builds_scene_and_keeps_success_result():
    """The workcell and placed target remain after a successful task."""
    calls = []

    execute_workspace_pick_place(
        FakeArm(calls),
        FakeGripper(calls),
        FakeScene(calls),
        _config(),
        sleep_fn=lambda duration: calls.append(('sleep', duration)),
    )

    assert calls == [
        ('joint', READY),
        ('open', 1.0),
        ('current_pose',),
        (
            'add_box', TABLE_ID, [0.70, 0.45, 0.30], [0.0, 0.48, 0.15],
            TABLE_COLOR,
        ),
        (
            'add_box', BACK_WALL_ID, [0.70, 0.05, 0.55],
            [0.0, 0.75, 0.575], BACK_WALL_COLOR,
        ),
        (
            'add_box', STORAGE_BOX_ID, [0.14, 0.14, 0.20],
            [-0.26, 0.62, 0.40], STORAGE_BOX_COLOR,
        ),
        (
            'add_box', CENTER_DIVIDER_ID, [0.06, 0.18, 0.24],
            [0.02, 0.47, 0.42], CENTER_DIVIDER_COLOR,
        ),
        (
            'add_box', TARGET_ID, [0.035, 0.035, 0.060],
            [0.16, 0.47, 0.34], TARGET_COLOR,
        ),
        ('sleep', 0.5),
        ('touch', TARGET_ID, GRIPPER_TOUCH_LINKS, True),
        ('pose', [0.16, 0.47, 0.41], [0.0, 0.0, 0.0, 1.0]),
        ('pose', [0.16, 0.47, 0.31], [0.0, 0.0, 0.0, 1.0]),
        ('gripper', 0.50, 1.0),
        ('attach', TARGET_ID, 'robotiq_tcp', GRIPPER_TOUCH_LINKS),
        ('pose', [0.16, 0.47, 0.41], [0.0, 0.0, 0.0, 1.0]),
        ('pose', [0.16, 0.32, 0.41], [0.0, 0.0, 0.0, 1.0]),
        ('pose', [0.16, 0.32, 0.31], [0.0, 0.0, 0.0, 1.0]),
        ('open', 1.0),
        ('detach', TARGET_ID),
        ('pose', [0.16, 0.32, 0.41], [0.0, 0.0, 0.0, 1.0]),
        ('touch', TARGET_ID, GRIPPER_TOUCH_LINKS, False),
        ('joint', READY),
    ]
    assert not any(call[0] == 'remove' for call in calls)


def test_static_scene_can_be_loaded_before_recovery_to_ready():
    """Physical simulations can plan the initial recovery around obstacles."""
    calls = []
    config = _config()
    config = WorkspacePickPlaceConfig(
        **{
            **config.__dict__,
            'static_scene_before_ready': True,
        }
    )

    execute_workspace_pick_place(
        FakeArm(calls),
        FakeGripper(calls),
        FakeScene(calls),
        config,
        sleep_fn=lambda duration: None,
    )

    assert [call[1] for call in calls[:4]] == [
        TABLE_ID,
        BACK_WALL_ID,
        STORAGE_BOX_ID,
        CENTER_DIVIDER_ID,
    ]
    assert calls[4] == ('joint', READY)


def test_pregrasp_failure_only_cleans_dynamic_target():
    """A failed route does not clear static objects or command later motion."""
    calls = []

    with pytest.raises(MotionPlanningError, match='blocked'):
        execute_workspace_pick_place(
            FakeArm(calls, fail_pose=1),
            FakeGripper(calls),
            FakeScene(calls),
            _config(),
            sleep_fn=lambda duration: None,
        )

    assert calls[-2:] == [
        ('touch', TARGET_ID, GRIPPER_TOUCH_LINKS, False),
        ('remove', TARGET_ID),
    ]
    assert ('attach', TARGET_ID, 'robotiq_tcp', GRIPPER_TOUCH_LINKS) not in calls
    assert calls.count(('joint', READY)) == 1


def test_transfer_failure_detaches_then_removes_dynamic_target():
    """An attached target is released before failure cleanup removes it."""
    calls = []

    with pytest.raises(MotionPlanningError, match='blocked'):
        execute_workspace_pick_place(
            FakeArm(calls, fail_pose=3),
            FakeGripper(calls),
            FakeScene(calls),
            _config(),
            sleep_fn=lambda duration: None,
        )

    assert calls[-3:] == [
        ('detach', TARGET_ID),
        ('touch', TARGET_ID, GRIPPER_TOUCH_LINKS, False),
        ('remove', TARGET_ID),
    ]
    assert calls.count(('joint', READY)) == 1


def test_gripper_close_failure_cleans_touch_permission_and_target():
    """A failed close never attaches the target or starts transport."""
    calls = []

    with pytest.raises(RuntimeError, match='gripper failed'):
        execute_workspace_pick_place(
            FakeArm(calls),
            FakeGripper(calls, fail_close=True),
            FakeScene(calls),
            _config(),
            sleep_fn=lambda duration: None,
        )

    assert ('attach', TARGET_ID, 'robotiq_tcp', GRIPPER_TOUCH_LINKS) not in calls
    assert calls[-2:] == [
        ('touch', TARGET_ID, GRIPPER_TOUCH_LINKS, False),
        ('remove', TARGET_ID),
    ]


@pytest.mark.parametrize(
    ('overrides', 'message'),
    [
        ({'table_size': [0.7, 0.45]}, 'exactly 3'),
        ({'target_size': [0.035, -0.01, 0.060]}, 'target_size'),
        ({'lift_translation': [0.0, 0.0, 0.0]}, 'positive Z'),
        ({'grasp_position': 0.81}, 'grasp_position'),
        ({'gripper_duration': 0.0}, 'gripper_duration'),
        ({'scene_wait_seconds': -0.1}, 'scene_wait_seconds'),
    ],
)
def test_invalid_workspace_parameters_are_rejected(overrides, message):
    """Invalid scene or motion values fail before MoveIt clients are created."""
    with pytest.raises(ValueError, match=message):
        _read_config(FakeParameterNode(overrides))
