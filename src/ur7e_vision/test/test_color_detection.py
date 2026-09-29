"""Unit tests for color segmentation, depth conversion and deprojection."""

import cv2
import numpy as np
import pytest

from ur7e_vision.color_detection import (
    create_hsv_mask,
    deproject_pixel,
    depth_to_meters,
    estimate_cuboid_pose,
    find_color_candidates,
    median_contour_depth,
    quaternion_rotation_matrix,
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


def test_known_cuboid_center_is_reconstructed_from_top_depth():
    depth = np.full((100, 100), np.nan, dtype=np.float32)
    depth[41:60, 41:60] = 0.9
    contour = np.array([[[41, 41]], [[59, 41]], [[59, 59]], [[41, 59]]])
    camera_to_base = np.eye(4)
    camera_to_base[:3, 3] = [0.1, 0.2, -0.54]

    result = estimate_cuboid_pose(
        depth, contour, (500.0, 500.0, 50.0, 50.0),
        camera_to_base, (0.035, 0.035, 0.060),
    )

    assert result is not None
    assert result.center == pytest.approx((0.1, 0.2, 0.33), abs=0.002)
    assert abs(result.yaw) < 0.01


def test_cuboid_fit_rejects_insufficient_visible_top():
    depth = np.full((100, 100), np.nan, dtype=np.float32)
    depth[48:53, 48:53] = 0.9
    contour = np.array([[[48, 48]], [[52, 48]], [[52, 52]], [[48, 52]]])

    assert estimate_cuboid_pose(
        depth, contour, (500.0, 500.0, 50.0, 50.0),
        np.eye(4), (0.035, 0.035, 0.060),
    ) is None


def test_quaternion_rotation_matrix_applies_camera_calibration():
    rotation = quaternion_rotation_matrix((0, 0, np.sqrt(0.5),
                                           np.sqrt(0.5)))
    np.testing.assert_allclose(rotation @ [1, 0, 0], [0, 1, 0], atol=1e-6)


@pytest.mark.parametrize('size,pixels', [
    ((0.035, 0.035, 0.060), (100, 100)),
    ((0.050, 0.025, 0.060), (139, 69)),
])
def test_rotated_top_face_estimates_object_x_axis(size, pixels):
    depth = np.full((240, 240), np.nan, dtype=np.float32)
    contour = cv2.boxPoints(((120, 120), pixels, 20.0))
    contour = np.round(contour).astype(np.int32).reshape(-1, 1, 2)
    mask = np.zeros(depth.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, cv2.FILLED)
    depth[mask > 0] = 0.9

    result = estimate_cuboid_pose(
        depth, contour, (2500.0, 2500.0, 120.0, 120.0),
        np.eye(4), size,
    )

    assert result is not None
    assert result.center == pytest.approx((0.0, 0.0, 0.87), abs=0.001)
    assert result.yaw == pytest.approx(np.deg2rad(20), abs=0.03)
