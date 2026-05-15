import math

import pytest

from xy_control.lk_total_transform_node import _bounded_yaw_correction
from xy_control.lk_total_transform_node import _grid_orientation_from_segments
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
