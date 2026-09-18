"""Tests for motion command parameter validation."""

import math

import pytest

from ur7e_motion.validation import (
    normalize_quaternion,
    validate_mode,
    validate_numeric_vector,
)


def test_validate_mode_normalizes_case_and_whitespace():
    """Mode values are normalized before dispatch."""
    assert validate_mode(" Joint ") == "joint"
    assert validate_mode("POSE") == "pose"


def test_validate_mode_rejects_unknown_mode():
    """Modes outside the two supported commands are rejected."""
    with pytest.raises(ValueError, match="mode must be one of"):
        validate_mode("cartesian")


def test_validate_numeric_vector_converts_values():
    """Numeric strings and integers are converted to floats."""
    assert validate_numeric_vector("translation", (0, "0.1", -0.2), 3) == [
        0.0,
        0.1,
        -0.2,
    ]


def test_validate_numeric_vector_rejects_wrong_length():
    """Vectors must have the mode-specific number of entries."""
    with pytest.raises(ValueError, match="exactly 6"):
        validate_numeric_vector("joint_positions", [0.0] * 5, 6)


@pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf])
def test_validate_numeric_vector_rejects_non_finite_values(invalid):
    """Non-finite values are never passed to MoveIt."""
    with pytest.raises(ValueError, match="finite"):
        validate_numeric_vector("translation", [0.0, 0.0, invalid], 3)


def test_normalize_quaternion_returns_unit_quaternion():
    """A non-unit quaternion is normalized before use as a pose goal."""
    assert normalize_quaternion([0.0, 0.0, 0.0, 2.0]) == [
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def test_normalize_quaternion_rejects_zero_norm():
    """A zero quaternion cannot describe an orientation."""
    with pytest.raises(ValueError, match="non-zero norm"):
        normalize_quaternion([0.0, 0.0, 0.0, 0.0])


def test_public_client_exposes_minimal_motion_api():
    """The reusable client exposes the intended motion and state methods."""
    from ur7e_motion import UR7eMotionError, UR7eMoveItClient

    assert callable(UR7eMoveItClient.move_to_joint)
    assert callable(UR7eMoveItClient.move_to_pose)
    assert callable(UR7eMoveItClient.move_by_translation)
    assert callable(UR7eMoveItClient.get_current_pose)
    assert issubclass(UR7eMotionError, RuntimeError)


def test_fixed_task_executes_steps_in_order():
    """The fixed task uses one client and follows its declared sequence."""
    from ur7e_motion.fixed_task import (
        MOVE_DOWN,
        MOVE_UP,
        MOVE_X_NEGATIVE,
        MOVE_X_POSITIVE,
        MOVE_Y_NEGATIVE,
        MOVE_Y_POSITIVE,
        READY,
        execute_task,
    )

    calls = []

    class FakeArm:
        def move_to_joint(self, positions):
            calls.append(("joint", positions))

        def move_by_translation(self, translation):
            calls.append(("translation", translation))

    execute_task(
        FakeArm(),
        pause_seconds=1.5,
        sleep_fn=lambda value: calls.append(("sleep", value)),
    )

    assert calls == [
        ("joint", READY),
        ("translation", MOVE_UP),
        ("sleep", 1.5),
        ("translation", MOVE_X_POSITIVE),
        ("sleep", 1.5),
        ("translation", MOVE_Y_POSITIVE),
        ("sleep", 1.5),
        ("translation", MOVE_X_NEGATIVE),
        ("sleep", 1.5),
        ("translation", MOVE_Y_NEGATIVE),
        ("sleep", 1.5),
        ("translation", MOVE_DOWN),
        ("sleep", 1.5),
        ("joint", READY),
    ]


def test_fixed_task_stops_after_motion_failure():
    """No later command is sent after one step raises an exception."""
    from ur7e_motion import MotionPlanningError
    from ur7e_motion.fixed_task import execute_task

    calls = []

    class FailingArm:
        def move_to_joint(self, positions):
            calls.append("joint")

        def move_by_translation(self, translation):
            calls.append("translation")
            raise MotionPlanningError("unreachable")

    with pytest.raises(MotionPlanningError, match="unreachable"):
        execute_task(FailingArm(), pause_seconds=0.0)

    assert calls == ["joint", "translation"]
