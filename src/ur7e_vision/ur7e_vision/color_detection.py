"""Pure OpenCV and geometry helpers used by the color detector node."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorCandidate:
    """A contour that has the expected projected shape of a colored cube."""

    center: tuple[int, int]
    bounding_box: tuple[int, int, int, int]
    area: float
    contour: np.ndarray


@dataclass(frozen=True)
class CuboidEstimate:
    """Known-size cuboid center and top-face yaw in the robot base frame."""

    center: tuple[float, float, float]
    yaw: float
    top_point_count: int


def quaternion_rotation_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """Return a 3x3 rotation matrix for a ROS XYZW quaternion."""
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('quaternion must contain four finite values')
    norm = np.linalg.norm(q)
    if norm <= 0.0:
        raise ValueError('quaternion must be nonzero')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def estimate_cuboid_pose(
    depth_meters: np.ndarray,
    contour: np.ndarray,
    intrinsics: Sequence[float],
    camera_to_base: np.ndarray,
    size: Sequence[float],
    *,
    min_depth: float = 0.1,
    max_depth: float = 3.0,
    top_band: float = 0.006,
    min_top_points: int = 20,
) -> CuboidEstimate | None:
    """Fit the visible top face; use only the known size to infer the center.

    ``camera_to_base`` is calibrated camera TF. No model/world pose or table
    coordinates enter this calculation. Reject a side-only or badly occluded
    observation instead of publishing an unsafe grasp target.
    """
    dimensions = np.asarray(size, dtype=np.float64)
    if dimensions.shape != (3,) or np.any(dimensions <= 0):
        raise ValueError('size must contain three positive values')
    if not np.all(np.isfinite(dimensions)):
        raise ValueError('size must contain finite values')
    transform = np.asarray(camera_to_base, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError('camera_to_base must be a finite 4x4 matrix')
    fx, fy, cx, cy = [float(value) for value in intrinsics]
    if fx <= 0 or fy <= 0:
        raise ValueError('focal lengths must be positive')

    contour_mask = np.zeros(depth_meters.shape[:2], dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
    contour_mask = cv2.erode(contour_mask, np.ones((3, 3), np.uint8))
    rows, columns = np.nonzero(
        (contour_mask > 0) & np.isfinite(depth_meters)
        & (depth_meters >= min_depth) & (depth_meters <= max_depth)
    )
    if rows.size < min_top_points:
        return None
    depths = depth_meters[rows, columns]
    optical_points = np.column_stack((
        (columns - cx) * depths / fx,
        (rows - cy) * depths / fy,
        depths,
    ))
    base_points = (
        optical_points @ transform[:3, :3].T + transform[:3, 3]
    )

    # Highest visible surface is the top face for an upright cuboid.
    top_z = float(np.percentile(base_points[:, 2], 95))
    top_points = base_points[
        np.abs(base_points[:, 2] - top_z) <= top_band
    ]
    if len(top_points) < min_top_points:
        return None
    rectangle = cv2.minAreaRect(top_points[:, :2].astype(np.float32))
    observed_sides = sorted(rectangle[1])
    expected_sides = sorted(dimensions[:2])
    if any(
        observed < expected * 0.5 or observed > expected * 1.5
        for observed, expected in zip(observed_sides, expected_sides)
    ):
        return None

    # A square has 90-degree yaw symmetry. Pick the equivalent yaw closest to
    # the base axes; the grasp planner may choose either opposite face pair.
    corners = cv2.boxPoints(rectangle)
    edge = corners[1] - corners[0]
    other_edge = corners[2] - corners[1]
    if abs(dimensions[0] - dimensions[1]) >= 1e-6:
        # 将物体 X 轴对应到已知 X 尺寸，避免长方形的朝向差 90 度。
        if (dimensions[0] > dimensions[1]) != (
            np.linalg.norm(edge) > np.linalg.norm(other_edge)
        ):
            edge = other_edge
    yaw = math.atan2(float(edge[1]), float(edge[0]))
    if abs(dimensions[0] - dimensions[1]) < 1e-6:
        yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    else:
        yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    return CuboidEstimate(
        (float(rectangle[0][0]), float(rectangle[0][1]),
         top_z - float(dimensions[2]) / 2),
        yaw,
        len(top_points),
    )


def create_hsv_mask(
    hsv_image: np.ndarray,
    ranges: Iterable[tuple[Sequence[int], Sequence[int]]],
    kernel_size: int = 5,
) -> np.ndarray:
    """Combine one or more HSV intervals and remove isolated pixel noise."""
    mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(
                hsv_image,
                np.asarray(lower, dtype=np.uint8),
                np.asarray(upper, dtype=np.uint8),
            ),
        )
    if kernel_size > 1:
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_color_candidates(
    mask: np.ndarray,
    min_area: float,
    max_area: float,
    min_aspect_ratio: float,
    max_aspect_ratio: float,
    min_fill_ratio: float,
) -> list[ColorCandidate]:
    """Return plausible cube contours, largest first."""
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    candidates: list[ColorCandidate] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area or area > max_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if height <= 0 or width <= 0:
            continue
        aspect_ratio = width / height
        fill_ratio = area / float(width * height)
        if not min_aspect_ratio <= aspect_ratio <= max_aspect_ratio:
            continue
        if fill_ratio < min_fill_ratio:
            continue
        moments = cv2.moments(contour)
        if moments['m00'] <= 0.0:
            continue
        center = (
            int(round(moments['m10'] / moments['m00'])),
            int(round(moments['m01'] / moments['m00'])),
        )
        candidates.append(
            ColorCandidate(center, (x, y, width, height), area, contour)
        )
    return sorted(candidates, key=lambda candidate: candidate.area, reverse=True)


def depth_to_meters(depth_image: np.ndarray, encoding: str) -> np.ndarray:
    """Convert common ROS depth encodings to floating-point metres."""
    if encoding == '32FC1':
        return depth_image.astype(np.float32, copy=False)
    if encoding == '16UC1':
        return depth_image.astype(np.float32) * 0.001
    raise ValueError(f'Unsupported depth encoding: {encoding}')


def median_contour_depth(
    depth_meters: np.ndarray,
    contour: np.ndarray,
    min_depth: float,
    max_depth: float,
) -> float | None:
    """Use the median valid depth inside a contour, rejecting edge outliers."""
    contour_mask = np.zeros(depth_meters.shape[:2], dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
    eroded = cv2.erode(contour_mask, np.ones((3, 3), np.uint8), iterations=1)
    selected = depth_meters[eroded > 0]
    valid = selected[
        np.isfinite(selected)
        & (selected >= min_depth)
        & (selected <= max_depth)
    ]
    if valid.size == 0:
        selected = depth_meters[contour_mask > 0]
        valid = selected[
            np.isfinite(selected)
            & (selected >= min_depth)
            & (selected <= max_depth)
        ]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def deproject_pixel(
    u: float,
    v: float,
    depth: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float, float]:
    """Convert a depth pixel into XYZ in the camera optical frame."""
    if depth <= 0.0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError('Depth and focal lengths must be positive')
    return (
        (u - cx) * depth / fx,
        (v - cy) * depth / fy,
        depth,
    )
