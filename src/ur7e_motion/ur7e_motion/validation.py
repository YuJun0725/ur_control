"""Validation helpers that do not require a running ROS graph."""

import math
from collections.abc import Sequence


VALID_MODES = ("joint", "pose")


def validate_mode(mode: str) -> str:
    """Return a normalized motion mode or raise ValueError."""
    normalized = mode.strip().lower()
    if normalized not in VALID_MODES:
        raise ValueError(
            f"mode must be one of {VALID_MODES}; received {mode!r}"
        )
    return normalized


def validate_numeric_vector(
    name: str, values: Sequence[float], expected_length: int
) -> list[float]:
    """Return finite floats after validating vector length and values."""
    if len(values) != expected_length:
        raise ValueError(
            f"{name} must contain exactly {expected_length} values; "
            f"received {len(values)}"
        )

    converted = [float(value) for value in values]
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{name} must contain only finite numbers")
    return converted


def normalize_quaternion(values: Sequence[float]) -> list[float]:
    """Validate and normalize a quaternion ordered as [x, y, z, w]."""
    quaternion = validate_numeric_vector("orientation", values, 4)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise ValueError("orientation quaternion must have a non-zero norm")
    return [value / norm for value in quaternion]
