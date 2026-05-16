import math

import numpy as np
import pytest

from xy_control.lk_total_transform_node import FeatureDetectionSettings
from xy_control.lk_total_transform_node import HoughGridObservation
from xy_control.lk_total_transform_node import LkTotalTransformNode
from xy_control.lk_total_transform_node import _adaptive_inlier_threshold
from xy_control.lk_total_transform_node import _adaptive_tracked_points_threshold
from xy_control.lk_total_transform_node import _bounded_yaw_correction
from xy_control.lk_total_transform_node import _detect_features_with_attempts
from xy_control.lk_total_transform_node import _feature_detection_attempts
from xy_control.lk_total_transform_node import _grid_orientation_from_segments
from xy_control.lk_total_transform_node import _hough_detection_attempts
from xy_control.lk_total_transform_node import _relative_grid_yaw
from xy_control.lk_total_transform_node import _select_nearest_grid_yaw
from xy_control.lk_total_transform_node import _wrap_to_period


def _segment(angle_rad: float, length: float = 100.0):
    return (
        0.0,
        0.0,
        length * math.cos(angle_rad),
        length * math.sin(angle_rad),
    )


def _assert_angle_close_mod(
    actual: float,
    expected: float,
    period: float = math.pi / 2.0,
    tol: float = 1e-6,
):
    assert abs(_wrap_to_period(actual - expected, period)) < tol


def test_perpendicular_grid_lines_fold_to_one_orientation():
    angle = math.radians(10.0)
    segments = [
        _segment(angle),
        _segment(angle + math.pi / 2.0),
        _segment(angle + math.pi),
        _segment(angle + 3.0 * math.pi / 2.0),
    ]

    observation = _grid_orientation_from_segments(segments, min_lines=4)

    assert observation is not None
    assert observation.line_count == 4
    assert observation.confidence == pytest.approx(1.0)
    _assert_angle_close_mod(observation.theta_rad, angle)


def test_two_large_grid_lines_can_make_adaptive_observation():
    angle = math.radians(15.0)

    observation = _grid_orientation_from_segments(
        [_segment(angle), _segment(angle + math.pi / 2.0)],
        min_lines=2,
    )

    assert observation is not None
    assert observation.line_count == 2
    assert observation.confidence == pytest.approx(1.0)
    _assert_angle_close_mod(observation.theta_rad, angle)


def test_reset_baseline_produces_zero_initial_hough_yaw():
    theta_reference = 0.32

    yaw = _relative_grid_yaw(theta_reference, theta_reference)

    assert yaw == pytest.approx(0.0)


def test_positive_grid_rotation_produces_negative_vehicle_yaw():
    theta_reference = -0.20

    yaw = _relative_grid_yaw(theta_reference + 0.12, theta_reference)

    assert yaw == pytest.approx(-0.12)


def test_nearest_grid_yaw_resolves_ninety_degree_branch():
    yaw_mod = 0.05
    lk_yaw = yaw_mod + math.pi / 2.0 + 0.08

    selected = _select_nearest_grid_yaw(yaw_mod, lk_yaw)

    assert selected == pytest.approx(yaw_mod + math.pi / 2.0)


def test_bounded_yaw_correction_limits_single_frame_step():
    corrected, step = _bounded_yaw_correction(
        current_yaw=0.0,
        measured_yaw=1.0,
        alpha=0.5,
        max_step_rad=0.03,
    )

    assert step == pytest.approx(0.03)
    assert corrected == pytest.approx(0.03)


def test_insufficient_or_low_confidence_lines_do_not_make_valid_measurement():
    insufficient = _grid_orientation_from_segments(
        [_segment(0.0)],
        min_lines=2,
    )
    low_confidence = _grid_orientation_from_segments(
        [_segment(0.0), _segment(math.pi / 4.0)],
        min_lines=2,
    )

    assert insufficient is None
    assert low_confidence is not None
    assert low_confidence.confidence < 0.55


def test_hough_none_does_not_change_lk_accumulated_yaw():
    node = LkTotalTransformNode.__new__(LkTotalTransformNode)
    node._enable_hough_yaw_correction = True
    node._last_hough_yaw = 0.4
    node._last_hough_error = 0.2
    node._last_hough_confidence = 0.9
    node._total_comp = LkTotalTransformNode._build_similarity(1.0, 2.0, 0.3, 1.0)

    before = node._total_comp.copy()

    LkTotalTransformNode._apply_hough_yaw_correction(node, None)

    assert np.allclose(node._total_comp, before)
    assert node._last_hough_yaw is None
    assert node._last_hough_error is None
    assert node._last_hough_confidence == pytest.approx(0.0)


def test_low_confidence_hough_does_not_force_yaw_correction():
    node = LkTotalTransformNode.__new__(LkTotalTransformNode)
    node._enable_hough_yaw_correction = True
    node._hough_min_confidence = 0.55
    node._last_hough_yaw = None
    node._last_hough_error = None
    node._last_hough_confidence = 0.0
    node._total_comp = LkTotalTransformNode._build_similarity(0.0, 0.0, 0.3, 1.0)
    observation = HoughGridObservation(
        theta_rad=0.0,
        confidence=0.2,
        line_count=2,
    )

    before = node._total_comp.copy()

    LkTotalTransformNode._apply_hough_yaw_correction(node, observation)

    assert np.allclose(node._total_comp, before)
    assert node._last_hough_confidence == pytest.approx(0.2)


def test_adaptive_tracked_threshold_allows_sparse_large_grid_features():
    threshold = _adaptive_tracked_points_threshold(
        prev_feature_count=40,
        min_tracked_points=100,
        enable_adaptive=True,
        floor=20,
        ratio=0.35,
    )

    assert threshold == 20
    assert _adaptive_tracked_points_threshold(40, 100, False, 20, 0.35) == 100


def test_adaptive_inlier_threshold_accepts_sparse_high_quality_tracks():
    sparse_threshold = _adaptive_inlier_threshold(
        good_track_count=20,
        min_inliers=10,
        enable_adaptive=True,
        floor=8,
        ratio=0.35,
    )
    denser_threshold = _adaptive_inlier_threshold(
        good_track_count=40,
        min_inliers=10,
        enable_adaptive=True,
        floor=8,
        ratio=0.35,
    )

    assert sparse_threshold == 8
    assert 7 < sparse_threshold
    assert denser_threshold == 14
    assert _adaptive_inlier_threshold(100, 10, True, 8, 0.35) == 20


def test_adaptive_feature_retry_uses_lower_quality_when_first_attempt_is_sparse():
    attempts = _feature_detection_attempts(
        quality_level=0.01,
        min_distance=7.0,
        adaptive_quality_levels=[0.01, 0.005, 0.002],
        min_distance_scale_min=0.5,
        enable_adaptive=True,
    )
    seen = []

    def detector(gray, settings: FeatureDetectionSettings):
        seen.append((settings.quality_level, settings.min_distance))
        if settings.quality_level <= 0.005 and settings.min_distance == 7.0:
            return np.zeros((20, 1, 2), dtype=np.float32)
        return np.zeros((5, 1, 2), dtype=np.float32)

    points = _detect_features_with_attempts(
        np.zeros((20, 20), dtype=np.uint8),
        attempts,
        target_count=20,
        detector=detector,
    )

    assert points is not None
    assert points.shape[0] == 20
    assert seen[:2] == [(0.01, 7.0), (0.005, 7.0)]


def test_adaptive_hough_attempts_end_at_configured_floors():
    attempts = _hough_detection_attempts(
        threshold=35,
        min_line_length_ratio=0.15,
        min_lines=6,
        enable_adaptive=True,
        threshold_floor=15,
        min_line_length_ratio_floor=0.08,
        min_lines_floor=2,
    )

    assert attempts[0].threshold == 35
    assert attempts[0].min_lines == 6
    assert attempts[0].adaptive_attempt is False
    assert attempts[-1].threshold == 15
    assert attempts[-1].min_line_length_ratio == pytest.approx(0.08)
    assert attempts[-1].min_lines == 2
    assert attempts[-1].adaptive_attempt is True
