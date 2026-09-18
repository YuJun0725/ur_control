"""Tests for the planning-scene obstacle demonstration."""

from types import SimpleNamespace

import pytest

from ur7e_motion import MotionPlanningError
from ur7e_motion.fixed_task import READY
from ur7e_motion.obstacle_demo import (
    OBSTACLE_ID,
    ObstacleDemoConfig,
    TABLE_ID,
    execute_obstacle_demo,
)


def _config() -> ObstacleDemoConfig:
    return ObstacleDemoConfig(
        table_size=[1.4, 1.4, 0.08],
        table_position=[0.0, 0.0, -0.05],
        obstacle_size=[0.04, 0.08, 0.08],
        obstacle_offset=[0.10, 0.0, 0.0],
        target_translation=[0.20, 0.0, 0.0],
        scene_wait_seconds=1.0,
        goal_hold_seconds=2.0,
    )


class FakeArm:
    """Record calls made by the demo without requiring MoveIt."""

    def __init__(self, calls):
        """Store the shared call log."""
        self.calls = calls

    def move_to_joint(self, positions):
        """Record a joint goal."""
        self.calls.append(("joint", positions))

    def get_current_pose(self):
        """Return a deterministic fake tool pose."""
        self.calls.append(("current_pose",))
        return SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.40, y=-0.20, z=0.50)
            )
        )

    def move_by_translation(self, translation):
        """Record a relative translation."""
        self.calls.append(("translation", translation))


class FakeScene:
    """Record scene changes made by the demo."""

    def __init__(self, calls):
        """Store the shared call log."""
        self.calls = calls

    def add_box(self, object_id, size, position):
        """Record an added collision box."""
        self.calls.append(("add_box", object_id, size, position))

    def clear(self):
        """Record scene cleanup."""
        self.calls.append(("clear",))


def test_obstacle_demo_places_box_between_tool_and_goal():
    """The obstacle position is computed from the latest tool pose."""
    calls = []
    execute_obstacle_demo(
        FakeArm(calls),
        FakeScene(calls),
        _config(),
        sleep_fn=lambda value: calls.append(("sleep", value)),
    )

    assert calls == [
        ("joint", READY),
        ("current_pose",),
        ("add_box", TABLE_ID, [1.4, 1.4, 0.08], [0.0, 0.0, -0.05]),
        (
            "add_box",
            OBSTACLE_ID,
            [0.04, 0.08, 0.08],
            [0.50, -0.20, 0.50],
        ),
        ("sleep", 1.0),
        ("translation", [0.20, 0.0, 0.0]),
        ("sleep", 2.0),
        ("joint", READY),
        ("clear",),
    ]


def test_obstacle_demo_clears_scene_after_motion_failure():
    """A failed motion does not leave demo objects in the scene."""
    calls = []

    class FailingArm(FakeArm):
        def move_by_translation(self, translation):
            """Fail when the demo requests its avoidance motion."""
            self.calls.append(("translation", translation))
            raise MotionPlanningError("blocked")

    with pytest.raises(MotionPlanningError, match="blocked"):
        execute_obstacle_demo(
            FailingArm(calls),
            FakeScene(calls),
            _config(),
            sleep_fn=lambda value: calls.append(("sleep", value)),
        )

    assert calls[-1] == ("clear",)
    assert ("joint", READY) not in calls[1:]
