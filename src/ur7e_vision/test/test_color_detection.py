"""Unit tests for color segmentation, depth conversion and deprojection."""

import cv2
import numpy as np
import pytest

from ur7e_vision.color_detection import (
    create_hsv_mask,
    deproject_pixel,
    depth_to_meters,
    find_color_candidates,
    median_contour_depth,
)


def test_detects_synthetic_red_cube_and_reads_median_depth():
    image = np.zeros((160, 220, 3), dtype=np.uint8)
    cv2.rectangle(image, (80, 50), (120, 100), (0, 0, 255), thickness=-1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = create_hsv_mask(
        hsv,
        [((0, 100, 60), (12, 255, 255)), ((168, 100, 60), (179, 255, 255))],
    )
    candidates = find_color_candidates(mask, 100, 3000, 0.4, 1.8, 0.4)

    assert len(candidates) == 1
    assert candidates[0].center == (100, 75)
    depth = np.full((160, 220), np.inf, dtype=np.float32)
    depth[50:101, 80:121] = 0.82
    assert median_contour_depth(depth, candidates[0].contour, 0.1, 3.0) == pytest.approx(0.82)


def test_depth_encoding_conversion_and_pixel_deprojection():
    millimetres = np.array([[750, 1000]], dtype=np.uint16)
    meters = depth_to_meters(millimetres, '16UC1')
    np.testing.assert_allclose(meters, [[0.75, 1.0]])
    assert deproject_pixel(330, 250, 1.0, 500, 500, 320, 240) == pytest.approx(
        (0.02, 0.02, 1.0)
    )


def test_rejects_unsupported_depth_encoding():
    with pytest.raises(ValueError, match='Unsupported depth encoding'):
        depth_to_meters(np.zeros((2, 2), dtype=np.uint8), '8UC1')
