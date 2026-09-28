"""Pure OpenCV and geometry helpers used by the color detector node."""

from dataclasses import dataclass
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
