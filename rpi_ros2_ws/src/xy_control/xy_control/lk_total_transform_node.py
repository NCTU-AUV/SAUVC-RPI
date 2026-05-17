#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger


def _T(tx: float, ty: float) -> np.ndarray:
    """3x3 translation matrix."""
    return np.array([
        [1.0, 0.0, tx],
        [0.0, 1.0, ty],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


@dataclass(frozen=True)
class HoughGridObservation:
    """Folded square-grid line orientation estimate."""

    theta_rad: float
    confidence: float
    line_count: int
    segments: Tuple[Tuple[float, float, float, float], ...] = ()
    hough_threshold: int = 0
    min_line_length_px: int = 0
    min_lines_required: int = 0
    adaptive_attempt: bool = False


@dataclass(frozen=True)
class FeatureDetectionSettings:
    quality_level: float
    min_distance: float
    adaptive_attempt: bool = False


@dataclass(frozen=True)
class HoughDetectionSettings:
    threshold: int
    min_line_length_ratio: float
    min_lines: int
    adaptive_attempt: bool = False


def _wrap_to_period(angle: float, period: float) -> float:
    return (angle + 0.5 * period) % period - 0.5 * period


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _debug_publish_period_ns(rate_hz: float) -> int:
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        return 0
    return int(round(1_000_000_000.0 / rate_hz))


def _should_publish_debug_image(
    now_ns: int,
    last_publish_ns: Optional[int],
    period_ns: int,
) -> bool:
    if period_ns <= 0 or last_publish_ns is None:
        return True
    return now_ns - last_publish_ns >= period_ns


def _resize_debug_overlay(
    overlay: np.ndarray,
    width_px: int,
    height_px: int,
) -> np.ndarray:
    if width_px <= 0 or height_px <= 0:
        return overlay

    h, w = overlay.shape[:2]
    target_size = (max(1, int(width_px)), max(1, int(height_px)))
    if (w, h) == target_size:
        return overlay

    interpolation = cv2.INTER_AREA
    if target_size[0] > w or target_size[1] > h:
        interpolation = cv2.INTER_LINEAR
    return cv2.resize(overlay, target_size, interpolation=interpolation)


def _unique_floats(values: Iterable[float], precision: int = 9) -> Tuple[float, ...]:
    unique = []
    seen = set()
    for value in values:
        if not math.isfinite(value) or value <= 0.0:
            continue
        key = round(float(value), precision)
        if key in seen:
            continue
        seen.add(key)
        unique.append(float(value))
    return tuple(unique)


def _as_float_sequence(value, fallback: Sequence[float]) -> Tuple[float, ...]:
    if isinstance(value, (str, bytes)):
        return tuple(float(v) for v in fallback)
    try:
        return tuple(float(v) for v in value)
    except (TypeError, ValueError):
        try:
            return (float(value),)
        except (TypeError, ValueError):
            return tuple(float(v) for v in fallback)


def _feature_detection_attempts(
    quality_level: float,
    min_distance: float,
    adaptive_quality_levels: Sequence[float],
    min_distance_scale_min: float,
    enable_adaptive: bool,
) -> Tuple[FeatureDetectionSettings, ...]:
    base = FeatureDetectionSettings(float(quality_level), float(min_distance), False)
    if not enable_adaptive:
        return (base,)

    qualities = _unique_floats(
        (
            quality_level,
            *(
                quality
                for quality in adaptive_quality_levels
                if float(quality) <= float(quality_level)
            ),
        )
    )
    if not qualities:
        qualities = (float(quality_level),)

    scale = _clamp(float(min_distance_scale_min), 0.0, 1.0)
    reduced_distance = float(min_distance) * scale
    distances = _unique_floats((min_distance, reduced_distance))
    if not distances:
        distances = (float(min_distance),)

    attempts = []
    seen = set()
    for distance_index, distance in enumerate(distances):
        for quality in qualities:
            key = (round(quality, 9), round(distance, 9))
            if key in seen:
                continue
            seen.add(key)
            attempts.append(
                FeatureDetectionSettings(
                    quality,
                    distance,
                    adaptive_attempt=distance_index > 0 or key != (
                        round(float(quality_level), 9),
                        round(float(min_distance), 9),
                    ),
                )
            )
    return tuple(attempts)


def _count_points(points: Optional[np.ndarray]) -> int:
    if points is None:
        return 0
    return int(points.reshape(-1, 2).shape[0])


def _detect_features_with_attempts(
    gray_small: np.ndarray,
    attempts: Sequence[FeatureDetectionSettings],
    target_count: int,
    detector: Callable[[np.ndarray, FeatureDetectionSettings], Optional[np.ndarray]],
) -> Optional[np.ndarray]:
    best_points = None
    best_count = 0
    for settings in attempts:
        points = detector(gray_small, settings)
        point_count = _count_points(points)
        if point_count > best_count:
            best_points = points
            best_count = point_count
        if point_count >= target_count:
            return points.astype(np.float32)
    if best_points is None:
        return None
    return best_points.astype(np.float32)


def _adaptive_tracked_points_threshold(
    prev_feature_count: int,
    min_tracked_points: int,
    enable_adaptive: bool,
    floor: int,
    ratio: float,
) -> int:
    if not enable_adaptive:
        return max(1, int(min_tracked_points))
    ratio_count = int(round(max(0, int(prev_feature_count)) * max(0.0, float(ratio))))
    return max(1, int(floor), min(int(min_tracked_points), ratio_count))


def _adaptive_inlier_threshold(
    good_track_count: int,
    min_inliers: int,
    enable_adaptive: bool,
    floor: int,
    ratio: float,
) -> int:
    if not enable_adaptive:
        return max(1, int(min_inliers))
    ratio_count = int(round(max(0, int(good_track_count)) * max(0.0, float(ratio))))
    dynamic = max(1, int(floor), ratio_count)
    cap = max(int(floor), int(min_inliers) * 2)
    return min(dynamic, cap)


def _hough_detection_attempts(
    threshold: int,
    min_line_length_ratio: float,
    min_lines: int,
    enable_adaptive: bool,
    threshold_floor: int,
    min_line_length_ratio_floor: float,
    min_lines_floor: int,
) -> Tuple[HoughDetectionSettings, ...]:
    base = HoughDetectionSettings(
        max(1, int(threshold)),
        max(0.0, float(min_line_length_ratio)),
        max(1, int(min_lines)),
        False,
    )
    if not enable_adaptive:
        return (base,)

    threshold_floor = max(1, min(base.threshold, int(threshold_floor)))
    ratio_floor = max(0.0, min(base.min_line_length_ratio, float(min_line_length_ratio_floor)))
    min_lines_floor = max(1, min(base.min_lines, int(min_lines_floor)))

    midpoint = HoughDetectionSettings(
        max(threshold_floor, int(round((base.threshold + threshold_floor) * 0.5))),
        max(ratio_floor, (base.min_line_length_ratio + ratio_floor) * 0.5),
        max(min_lines_floor, int(round((base.min_lines + min_lines_floor) * 0.5))),
        True,
    )
    floor = HoughDetectionSettings(
        threshold_floor,
        ratio_floor,
        min_lines_floor,
        True,
    )

    attempts = []
    seen = set()
    for settings in (base, midpoint, floor):
        key = (
            settings.threshold,
            round(settings.min_line_length_ratio, 9),
            settings.min_lines,
        )
        if key in seen:
            continue
        seen.add(key)
        attempts.append(settings)
    return tuple(attempts)


def _grid_orientation_from_segments(
    segments: Iterable[Tuple[float, float, float, float]],
    min_lines: int,
    hough_threshold: int = 0,
    min_line_length_px: int = 0,
    adaptive_attempt: bool = False,
) -> Optional[HoughGridObservation]:
    total_weight = 0.0
    sum_cos = 0.0
    sum_sin = 0.0
    line_count = 0
    valid_segments = []

    for x1, y1, x2, y2 in segments:
        dx = float(x2) - float(x1)
        dy = float(y2) - float(y1)
        length = math.hypot(dx, dy)
        if length <= 0.0:
            continue

        angle = math.atan2(dy, dx)
        folded = 4.0 * angle
        sum_cos += length * math.cos(folded)
        sum_sin += length * math.sin(folded)
        total_weight += length
        line_count += 1
        valid_segments.append((float(x1), float(y1), float(x2), float(y2)))

    if line_count < min_lines or total_weight <= 0.0:
        return None

    theta = 0.25 * math.atan2(sum_sin, sum_cos)
    theta = _wrap_to_period(theta, math.pi / 2.0)
    confidence = math.hypot(sum_cos, sum_sin) / total_weight
    return HoughGridObservation(
        theta,
        confidence,
        line_count,
        tuple(valid_segments),
        int(hough_threshold),
        int(min_line_length_px),
        int(min_lines),
        bool(adaptive_attempt),
    )


def _map_line_angle(angle_rad: float, mapping: np.ndarray) -> float:
    direction = np.array([
        [math.cos(angle_rad)],
        [math.sin(angle_rad)],
    ], dtype=np.float64)
    mapped = mapping @ direction
    return math.atan2(float(mapped[1, 0]), float(mapped[0, 0]))


def _relative_grid_yaw(theta_now: float, theta_reference: float) -> float:
    return -_wrap_to_period(theta_now - theta_reference, math.pi / 2.0)


def _select_nearest_grid_yaw(yaw_mod: float, reference_yaw: float) -> float:
    period = math.pi / 2.0
    nearest_k = int(round((reference_yaw - yaw_mod) / period))
    candidates = [
        yaw_mod + (nearest_k + offset) * period
        for offset in (-1, 0, 1)
    ]
    return min(candidates, key=lambda yaw: abs(yaw - reference_yaw))


def _bounded_yaw_correction(
    current_yaw: float,
    measured_yaw: float,
    alpha: float,
    max_step_rad: float,
) -> Tuple[float, float]:
    error = _wrap_to_pi(measured_yaw - current_yaw)
    step = alpha * error
    if max_step_rad >= 0.0:
        step = _clamp(step, -max_step_rad, max_step_rad)
    return current_yaw + step, step


class LkTotalTransformNode(Node):
    """Track bottom-camera image motion and publish accumulated pose."""

    def __init__(self):
        super().__init__('lk_total_transform_node')

        # ---- Params (topics) ----
        self.declare_parameter('image_topic', 'camera/bottom/image_raw')
        self.declare_parameter('output_topic', 'camera/bottom/pose_px')
        self.declare_parameter('output_x_topic', 'control/pid/bottom_camera/x/feedback_px')
        self.declare_parameter('output_y_topic', 'control/pid/bottom_camera/y/feedback_px')
        self.declare_parameter('output_yaw_topic', 'state/bottom_camera/yaw_rad')
        self.declare_parameter('output_scale_topic', 'state/bottom_camera/scale')

        # Hough grid yaw diagnostics
        self.declare_parameter(
            'output_yaw_lk_uncorrected_topic',
            'state/bottom_camera/yaw_lk_uncorrected_rad',
        )
        self.declare_parameter(
            'output_yaw_hough_topic',
            'state/bottom_camera/yaw_hough_rad',
        )
        self.declare_parameter(
            'output_yaw_hough_error_topic',
            'state/bottom_camera/yaw_hough_error_rad',
        )
        self.declare_parameter(
            'output_yaw_hough_confidence_topic',
            'state/bottom_camera/yaw_hough_confidence',
        )

        # raw output (debug / diagnostics)
        self.declare_parameter('publish_raw_output', True)
        self.declare_parameter('output_topic_raw', 'camera/bottom/pose_raw_px')

        # debug image overlay
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', 'camera/bottom/debug/lk_tracks')
        self.declare_parameter('publish_hough_debug_image', False)
        self.declare_parameter(
            'hough_debug_image_topic',
            'camera/bottom/debug/hough_lines',
        )
        self.declare_parameter('debug_publish_rate_hz', 5.0)
        self.declare_parameter('debug_output_width_px', 80)
        self.declare_parameter('debug_output_height_px', 60)

        # ---- Params (rotation center) ----
        # 'image_center' or 'principal_point'
        self.declare_parameter('rotation_center_mode', 'image_center')
        self.declare_parameter('camera_info_topic', 'camera/bottom/camera_info')

        # ---- Params (image -> body mapping) ----
        self.declare_parameter('image_to_body_yaw_deg', 90.0)
        self.declare_parameter('flip_image_x', False)
        self.declare_parameter('flip_image_y', False)

        # ---- Params (image scale) ----
        self.declare_parameter('resize_width_px', 320)

        # ---- Params (feature detection) ----
        self.declare_parameter('max_corners', 500)
        self.declare_parameter('quality_level', 0.01)
        self.declare_parameter('min_distance', 7.0)
        self.declare_parameter('block_size', 7)

        # ---- Params (LK / PyrLK) ----
        self.declare_parameter('win_size', 21)
        self.declare_parameter('max_level', 2)
        self.declare_parameter('criteria_count', 20)
        self.declare_parameter('criteria_eps', 0.03)
        self.declare_parameter('max_track_error', 50.0)  # <=0 to disable

        # ---- Params (robust estimation) ----
        self.declare_parameter('ransac_reproj_threshold_px', 3.0)
        self.declare_parameter('min_inliers', 10)
        self.declare_parameter('min_tracked_points', 100)
        self.declare_parameter('reinit_if_fail', True)

        # ---- Params (adaptive detection for low-feature pool floors) ----
        self.declare_parameter('enable_adaptive_detection', True)
        self.declare_parameter('adaptive_min_tracked_points_floor', 20)
        self.declare_parameter('adaptive_min_inliers_floor', 8)
        self.declare_parameter('adaptive_min_inlier_ratio', 0.35)
        self.declare_parameter(
            'adaptive_feature_quality_levels',
            [0.01, 0.005, 0.002, 0.001],
        )
        self.declare_parameter('adaptive_min_distance_scale_min', 0.5)
        self.declare_parameter('adaptive_hough_min_lines_floor', 2)
        self.declare_parameter('adaptive_hough_threshold_floor', 15)
        self.declare_parameter('adaptive_hough_min_line_length_ratio_floor', 0.08)

        # ---- Params (Hough square-grid yaw correction) ----
        self.declare_parameter('enable_hough_yaw_correction', True)
        self.declare_parameter('hough_correction_alpha', 0.15)
        self.declare_parameter('hough_max_correction_step_rad', 0.03)
        self.declare_parameter('hough_min_lines', 6)
        self.declare_parameter('hough_min_confidence', 0.55)
        self.declare_parameter('hough_reject_error_rad', 0.70)
        self.declare_parameter('hough_canny_low', 50)
        self.declare_parameter('hough_canny_high', 150)
        self.declare_parameter('hough_threshold', 35)
        self.declare_parameter('hough_min_line_length_ratio', 0.15)
        self.declare_parameter('hough_max_line_gap_px', 10)

        # ---- Load params ----
        self._bridge = CvBridge()
        self._image_topic = self.get_parameter('image_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._output_x_topic = self.get_parameter('output_x_topic').value
        self._output_y_topic = self.get_parameter('output_y_topic').value
        self._output_yaw_topic = self.get_parameter('output_yaw_topic').value
        self._output_scale_topic = self.get_parameter('output_scale_topic').value
        self._output_yaw_lk_uncorrected_topic = self.get_parameter(
            'output_yaw_lk_uncorrected_topic'
        ).value
        self._output_yaw_hough_topic = self.get_parameter(
            'output_yaw_hough_topic'
        ).value
        self._output_yaw_hough_error_topic = self.get_parameter(
            'output_yaw_hough_error_topic'
        ).value
        self._output_yaw_hough_confidence_topic = self.get_parameter(
            'output_yaw_hough_confidence_topic'
        ).value

        self._publish_raw_output = bool(self.get_parameter('publish_raw_output').value)
        self._output_topic_raw = self.get_parameter('output_topic_raw').value

        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._debug_image_topic = self.get_parameter('debug_image_topic').value
        self._publish_hough_debug_image = bool(
            self.get_parameter('publish_hough_debug_image').value
        )
        self._hough_debug_image_topic = self.get_parameter(
            'hough_debug_image_topic'
        ).value
        self._debug_publish_period_ns = _debug_publish_period_ns(
            float(self.get_parameter('debug_publish_rate_hz').value)
        )
        self._debug_output_width_px = int(
            self.get_parameter('debug_output_width_px').value
        )
        self._debug_output_height_px = int(
            self.get_parameter('debug_output_height_px').value
        )

        self._rotation_center_mode = str(self.get_parameter('rotation_center_mode').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        yaw_deg = float(self.get_parameter('image_to_body_yaw_deg').value)
        flip_x = bool(self.get_parameter('flip_image_x').value)
        flip_y = bool(self.get_parameter('flip_image_y').value)
        self._image_to_body = self._build_mapping_matrix(yaw_deg, flip_x, flip_y)
        self._body_to_image = np.linalg.inv(self._image_to_body)

        self._resize_width_px = int(self.get_parameter('resize_width_px').value)

        self._max_corners = int(self.get_parameter('max_corners').value)
        self._quality_level = float(self.get_parameter('quality_level').value)
        self._min_distance = float(self.get_parameter('min_distance').value)
        self._block_size = int(self.get_parameter('block_size').value)

        self._win_size = int(self.get_parameter('win_size').value)
        self._max_level = int(self.get_parameter('max_level').value)
        self._criteria_count = int(self.get_parameter('criteria_count').value)
        self._criteria_eps = float(self.get_parameter('criteria_eps').value)
        self._max_track_error = float(self.get_parameter('max_track_error').value)
        self._lk_criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            self._criteria_count,
            self._criteria_eps,
        )

        self._ransac_thresh = float(self.get_parameter('ransac_reproj_threshold_px').value)
        self._min_inliers = int(self.get_parameter('min_inliers').value)
        self._min_tracked_points = int(self.get_parameter('min_tracked_points').value)
        self._reinit_if_fail = bool(self.get_parameter('reinit_if_fail').value)

        self._enable_adaptive_detection = bool(
            self.get_parameter('enable_adaptive_detection').value
        )
        self._adaptive_min_tracked_points_floor = int(
            self.get_parameter('adaptive_min_tracked_points_floor').value
        )
        self._adaptive_min_inliers_floor = int(
            self.get_parameter('adaptive_min_inliers_floor').value
        )
        self._adaptive_min_inlier_ratio = float(
            self.get_parameter('adaptive_min_inlier_ratio').value
        )
        quality_levels_value = self.get_parameter(
            'adaptive_feature_quality_levels'
        ).value
        self._adaptive_feature_quality_levels = _unique_floats(
            _as_float_sequence(
                quality_levels_value,
                [0.01, 0.005, 0.002, 0.001],
            )
        )
        if not self._adaptive_feature_quality_levels:
            self._adaptive_feature_quality_levels = (self._quality_level,)
        self._adaptive_min_distance_scale_min = float(
            self.get_parameter('adaptive_min_distance_scale_min').value
        )
        self._adaptive_hough_min_lines_floor = int(
            self.get_parameter('adaptive_hough_min_lines_floor').value
        )
        self._adaptive_hough_threshold_floor = int(
            self.get_parameter('adaptive_hough_threshold_floor').value
        )
        self._adaptive_hough_min_line_length_ratio_floor = float(
            self.get_parameter('adaptive_hough_min_line_length_ratio_floor').value
        )

        self._enable_hough_yaw_correction = bool(
            self.get_parameter('enable_hough_yaw_correction').value
        )
        self._hough_correction_alpha = float(
            self.get_parameter('hough_correction_alpha').value
        )
        self._hough_max_correction_step_rad = float(
            self.get_parameter('hough_max_correction_step_rad').value
        )
        self._hough_min_lines = int(self.get_parameter('hough_min_lines').value)
        self._hough_min_confidence = float(
            self.get_parameter('hough_min_confidence').value
        )
        self._hough_reject_error_rad = float(
            self.get_parameter('hough_reject_error_rad').value
        )
        self._hough_canny_low = int(self.get_parameter('hough_canny_low').value)
        self._hough_canny_high = int(self.get_parameter('hough_canny_high').value)
        self._hough_threshold = int(self.get_parameter('hough_threshold').value)
        self._hough_min_line_length_ratio = float(
            self.get_parameter('hough_min_line_length_ratio').value
        )
        self._hough_max_line_gap_px = int(
            self.get_parameter('hough_max_line_gap_px').value
        )

        # ---- CameraInfo principal point (optional) ----
        self._have_pp = False
        self._pp_cx = 0.0
        self._pp_cy = 0.0

        # ---- State ----
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None  # (N,1,2) float32 on downscaled image

        # running totals (vehicle pose in world frame)
        self._total_raw = np.eye(3, dtype=np.float64)
        self._total_comp_lk = np.eye(3, dtype=np.float64)
        self._total_comp = np.eye(3, dtype=np.float64)

        # Hough grid yaw correction state
        self._hough_reference_body_rad: Optional[float] = None
        self._last_hough_yaw: Optional[float] = None
        self._last_hough_error: Optional[float] = None
        self._last_hough_confidence = 0.0
        self._last_hough_settings: Optional[HoughDetectionSettings] = None
        self._last_lk_debug_publish_ns: Optional[int] = None
        self._last_hough_debug_publish_ns: Optional[int] = None

        # ---- Pub/Sub ----
        self._pub_comp = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._pub_x = self.create_publisher(Float64, self._output_x_topic, 10)
        self._pub_y = self.create_publisher(Float64, self._output_y_topic, 10)
        self._pub_yaw = self.create_publisher(Float64, self._output_yaw_topic, 10)
        self._pub_scale = self.create_publisher(Float64, self._output_scale_topic, 10)
        self._pub_yaw_lk_uncorrected = self.create_publisher(
            Float64,
            self._output_yaw_lk_uncorrected_topic,
            10,
        )
        self._pub_yaw_hough = self.create_publisher(
            Float64,
            self._output_yaw_hough_topic,
            10,
        )
        self._pub_yaw_hough_error = self.create_publisher(
            Float64,
            self._output_yaw_hough_error_topic,
            10,
        )
        self._pub_yaw_hough_confidence = self.create_publisher(
            Float64,
            self._output_yaw_hough_confidence_topic,
            10,
        )
        self._pub_raw = None
        if self._publish_raw_output:
            self._pub_raw = self.create_publisher(Float64MultiArray, self._output_topic_raw, 10)

        self._debug_pub = None
        if self._publish_debug_image:
            self._debug_pub = self.create_publisher(Image, self._debug_image_topic, 10)
        self._hough_debug_pub = None
        if self._publish_hough_debug_image:
            self._hough_debug_pub = self.create_publisher(
                Image,
                self._hough_debug_image_topic,
                10,
            )

        self._sub_img = self.create_subscription(Image, self._image_topic, self._on_image, 10)
        self._reset_service = self.create_service(
            Trigger,
            'camera/bottom/reset_pose',
            self._on_reset_pose,
        )
        self._sub_info = None
        if self._rotation_center_mode == 'principal_point':
            self._sub_info = self.create_subscription(
                CameraInfo,
                self._camera_info_topic,
                self._on_camerainfo,
                10,
            )

        self.get_logger().info(
            f'LK total transform node started. image_topic={self._image_topic}, '
            f'output_topic={self._output_topic}, '
            f'rotation_center_mode={self._rotation_center_mode}, '
            f'publish_raw={self._publish_raw_output}\n'
            f'scalar outputs: x={self._output_x_topic}, y={self._output_y_topic}, '
            f'yaw={self._output_yaw_topic}, scale={self._output_scale_topic}, '
            f'hough_yaw_correction={self._enable_hough_yaw_correction}, '
            f'hough_debug={self._hough_debug_image_topic}, '
            f'adaptive_detection={self._enable_adaptive_detection}'
        )

    def _on_camerainfo(self, msg: CameraInfo):
        # CameraInfo.K = [fx,0,cx, 0,fy,cy, 0,0,1]
        if len(msg.k) >= 6:
            cx = float(msg.k[2])
            cy = float(msg.k[5])
            if math.isfinite(cx) and math.isfinite(cy):
                self._pp_cx = cx
                self._pp_cy = cy
                self._have_pp = True

    def _to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}', throttle_duration_sec=5.0)
            return None

    def _downscale_gray(self, gray: np.ndarray) -> Tuple[np.ndarray, float]:
        h, w = gray.shape[:2]
        if self._resize_width_px <= 0 or w <= self._resize_width_px:
            return gray, 1.0
        scale = float(self._resize_width_px) / float(w)
        new_w = self._resize_width_px
        new_h = max(1, int(round(h * scale)))
        small = cv2.resize(gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
        return small, scale

    def _feature_detection_target(self) -> int:
        if not self._enable_adaptive_detection:
            return max(1, self._min_tracked_points)
        return max(
            1,
            min(self._max_corners, self._adaptive_min_tracked_points_floor),
        )

    def _detect_features_once(
        self,
        gray_small: np.ndarray,
        settings: FeatureDetectionSettings,
    ) -> Optional[np.ndarray]:
        pts = cv2.goodFeaturesToTrack(
            gray_small,
            mask=None,
            maxCorners=self._max_corners,
            qualityLevel=settings.quality_level,
            minDistance=settings.min_distance,
            blockSize=self._block_size,
        )
        if pts is None:
            return None
        return pts.astype(np.float32)

    def _detect_features(self, gray_small: np.ndarray) -> Optional[np.ndarray]:
        attempts = _feature_detection_attempts(
            self._quality_level,
            self._min_distance,
            self._adaptive_feature_quality_levels,
            self._adaptive_min_distance_scale_min,
            self._enable_adaptive_detection,
        )
        return _detect_features_with_attempts(
            gray_small,
            attempts,
            self._feature_detection_target(),
            self._detect_features_once,
        )

    def _tracked_points_threshold(self, prev_feature_count: int) -> int:
        return _adaptive_tracked_points_threshold(
            prev_feature_count,
            self._min_tracked_points,
            self._enable_adaptive_detection,
            self._adaptive_min_tracked_points_floor,
            self._adaptive_min_inlier_ratio,
        )

    def _inlier_threshold(self, good_track_count: int) -> int:
        return _adaptive_inlier_threshold(
            good_track_count,
            self._min_inliers,
            self._enable_adaptive_detection,
            self._adaptive_min_inliers_floor,
            self._adaptive_min_inlier_ratio,
        )

    @staticmethod
    def _extract_similarity(H: np.ndarray) -> Tuple[float, float, float, float]:
        tx = float(H[0, 2])
        ty = float(H[1, 2])
        a = float(H[0, 0])
        c = float(H[1, 0])
        scale = math.sqrt(a * a + c * c)
        rot = math.atan2(c, a)
        return tx, ty, rot, scale

    @staticmethod
    def _build_similarity(
        tx: float,
        ty: float,
        rot: float,
        scale: float,
    ) -> np.ndarray:
        cos_rot = math.cos(rot)
        sin_rot = math.sin(rot)
        return np.array([
            [scale * cos_rot, -scale * sin_rot, tx],
            [scale * sin_rot, scale * cos_rot, ty],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def _detect_hough_grid(
        self,
        gray_small: np.ndarray,
        settings: HoughDetectionSettings,
    ) -> Optional[HoughGridObservation]:
        if gray_small is None or gray_small.size == 0:
            return None

        blurred = cv2.GaussianBlur(gray_small, (5, 5), 0)
        edges = cv2.Canny(
            blurred,
            self._hough_canny_low,
            self._hough_canny_high,
            apertureSize=3,
        )

        h, w = gray_small.shape[:2]
        min_line_length = max(
            1,
            int(round(min(h, w) * settings.min_line_length_ratio)),
        )
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=settings.threshold,
            minLineLength=min_line_length,
            maxLineGap=self._hough_max_line_gap_px,
        )
        if lines is None:
            return None

        segments = [
            tuple(float(v) for v in line[0])
            for line in lines
        ]
        return _grid_orientation_from_segments(
            segments,
            min_lines=settings.min_lines,
            hough_threshold=settings.threshold,
            min_line_length_px=min_line_length,
            adaptive_attempt=settings.adaptive_attempt,
        )

    def _maybe_detect_hough_grid(
        self,
        gray_small: np.ndarray,
    ) -> Optional[HoughGridObservation]:
        if not (
            self._enable_hough_yaw_correction
            or self._publish_hough_debug_image
        ):
            return None
        self._last_hough_settings = None
        attempts = _hough_detection_attempts(
            self._hough_threshold,
            self._hough_min_line_length_ratio,
            self._hough_min_lines,
            self._enable_adaptive_detection,
            self._adaptive_hough_threshold_floor,
            self._adaptive_hough_min_line_length_ratio_floor,
            self._adaptive_hough_min_lines_floor,
        )
        for settings in attempts:
            self._last_hough_settings = settings
            observation = self._detect_hough_grid(gray_small, settings)
            if observation is not None:
                return observation
        return None

    def _hough_angle_to_body(self, theta_image: float) -> float:
        theta_body = _map_line_angle(theta_image, self._image_to_body)
        return _wrap_to_period(theta_body, math.pi / 2.0)

    def _apply_hough_yaw_correction(
        self,
        observation: Optional[HoughGridObservation],
    ) -> None:
        self._last_hough_yaw = None
        self._last_hough_error = None
        self._last_hough_confidence = 0.0

        if not self._enable_hough_yaw_correction:
            return

        if observation is None:
            return

        self._last_hough_confidence = float(observation.confidence)
        if observation.confidence < self._hough_min_confidence:
            return

        theta_body = self._hough_angle_to_body(observation.theta_rad)
        if self._hough_reference_body_rad is None:
            self._hough_reference_body_rad = theta_body

        hough_yaw_mod = _relative_grid_yaw(
            theta_body,
            self._hough_reference_body_rad,
        )
        _, _, lk_yaw, _ = self._extract_similarity(self._total_comp_lk)
        hough_yaw = _select_nearest_grid_yaw(hough_yaw_mod, lk_yaw)
        self._last_hough_yaw = hough_yaw

        tx, ty, current_yaw, scale = self._extract_similarity(self._total_comp)
        self._last_hough_error = _wrap_to_pi(hough_yaw - current_yaw)
        hough_lk_error = hough_yaw - lk_yaw
        if abs(hough_lk_error) > self._hough_reject_error_rad:
            return

        corrected_yaw, _ = _bounded_yaw_correction(
            current_yaw,
            hough_yaw,
            self._hough_correction_alpha,
            self._hough_max_correction_step_rad,
        )
        self._total_comp = self._build_similarity(tx, ty, corrected_yaw, scale)
        self._last_hough_error = _wrap_to_pi(hough_yaw - corrected_yaw)

    def _publish_hough_debug(
        self,
        frame_bgr: np.ndarray,
        observation: Optional[HoughGridObservation],
        scale_factor: float,
    ) -> None:
        if self._hough_debug_pub is None or frame_bgr is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if not _should_publish_debug_image(
            now_ns,
            self._last_hough_debug_publish_ns,
            self._debug_publish_period_ns,
        ):
            return

        overlay = frame_bgr.copy()
        if observation is None:
            if self._last_hough_settings is None:
                status_text = 'Hough: no lines'
            else:
                status_text = (
                    'Hough rejected: no lines '
                    f'thr={self._last_hough_settings.threshold} '
                    f'min_lines={self._last_hough_settings.min_lines}'
                )
            color = (0, 0, 255)
        else:
            accepted = observation.confidence >= self._hough_min_confidence
            color = (0, 255, 0) if accepted else (0, 255, 255)
            for x1, y1, x2, y2 in observation.segments:
                if scale_factor > 0 and scale_factor != 1.0:
                    x1 /= scale_factor
                    y1 /= scale_factor
                    x2 /= scale_factor
                    y2 /= scale_factor
                cv2.line(
                    overlay,
                    (int(round(x1)), int(round(y1))),
                    (int(round(x2)), int(round(y2))),
                    color,
                    2,
                )
            status = 'accepted' if accepted else 'rejected'
            adaptive = ' adaptive' if observation.adaptive_attempt else ''
            status_text = (
                f'Hough {status}{adaptive}: lines={observation.line_count} '
                f'conf={observation.confidence:.2f} '
                f'thr={observation.hough_threshold} '
                f'min_len={observation.min_line_length_px} '
                f'min_lines={observation.min_lines_required}'
            )

            h, w = overlay.shape[:2]
            cx = 0.5 * float(w)
            cy = 0.5 * float(h)
            axis_len = 0.35 * float(min(h, w))
            dx = axis_len * math.cos(observation.theta_rad)
            dy = axis_len * math.sin(observation.theta_rad)
            cv2.line(
                overlay,
                (int(round(cx - dx)), int(round(cy - dy))),
                (int(round(cx + dx)), int(round(cy + dy))),
                (255, 255, 0),
                2,
            )

        cv2.putText(
            overlay,
            status_text,
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

        overlay = _resize_debug_overlay(
            overlay,
            self._debug_output_width_px,
            self._debug_output_height_px,
        )
        try:
            out = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to convert Hough debug image: {exc}',
                throttle_duration_sec=5.0,
            )
            return
        self._hough_debug_pub.publish(out)
        self._last_hough_debug_publish_ns = now_ns

    def _publish(self):
        # compensated output (main topic)
        tx, ty, rot, scale = self._extract_similarity(self._total_comp)
        msg = Float64MultiArray()
        msg.data = [float(tx), float(ty), float(rot), float(scale)]
        self._pub_comp.publish(msg)
        self._publish_float(self._pub_x, tx)
        self._publish_float(self._pub_y, ty)
        self._publish_float(self._pub_yaw, rot)
        self._publish_float(self._pub_scale, scale)
        _, _, lk_rot, _ = self._extract_similarity(self._total_comp_lk)
        self._publish_float(self._pub_yaw_lk_uncorrected, lk_rot)
        self._publish_float(
            self._pub_yaw_hough,
            self._last_hough_yaw
            if self._last_hough_yaw is not None
            else math.nan,
        )
        self._publish_float(
            self._pub_yaw_hough_error,
            self._last_hough_error
            if self._last_hough_error is not None
            else math.nan,
        )
        self._publish_float(
            self._pub_yaw_hough_confidence,
            self._last_hough_confidence,
        )

        # raw output (optional diagnostics)
        if self._pub_raw is not None:
            tx2, ty2, rot2, scale2 = self._extract_similarity(self._total_raw)
            msg2 = Float64MultiArray()
            msg2.data = [float(tx2), float(ty2), float(rot2), float(scale2)]
            self._pub_raw.publish(msg2)

    def _on_reset_pose(self, request, response):
        self._prev_gray = None
        self._prev_pts = None
        self._total_raw = np.eye(3, dtype=np.float64)
        self._total_comp_lk = np.eye(3, dtype=np.float64)
        self._total_comp = np.eye(3, dtype=np.float64)
        self._hough_reference_body_rad = None
        self._last_hough_yaw = None
        self._last_hough_error = None
        self._last_hough_confidence = 0.0
        self._last_hough_settings = None
        self._publish()

        response.success = True
        response.message = "Bottom-camera pose reset"
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def _publish_debug(self, frame_bgr: np.ndarray, pts_small: np.ndarray, scale_factor: float):
        if self._debug_pub is None or frame_bgr is None or pts_small is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if not _should_publish_debug_image(
            now_ns,
            self._last_lk_debug_publish_ns,
            self._debug_publish_period_ns,
        ):
            return

        pts = pts_small.reshape(-1, 2)
        if scale_factor > 0 and scale_factor != 1.0:
            pts = pts / scale_factor

        overlay = frame_bgr.copy()
        for x, y in pts:
            cv2.circle(overlay, (int(x), int(y)), 3, (0, 0, 255), -1)

        overlay = _resize_debug_overlay(
            overlay,
            self._debug_output_width_px,
            self._debug_output_height_px,
        )
        try:
            out = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to convert debug image: {exc}',
                throttle_duration_sec=5.0,
            )
            return
        self._debug_pub.publish(out)
        self._last_lk_debug_publish_ns = now_ns

    def _get_rotation_center(self, w: int, h: int) -> Tuple[float, float]:
        # Use principal point if requested & available; else image center
        if self._rotation_center_mode == 'principal_point' and self._have_pp:
            return self._pp_cx, self._pp_cy
        return 0.5 * float(w), 0.5 * float(h)

    def _on_image(self, msg: Image):
        frame_bgr = self._to_bgr(msg)
        if frame_bgr is None:
            return

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        h_img, w_img = gray.shape[:2]

        gray_small, scale_factor = self._downscale_gray(gray)
        hough_observation = self._maybe_detect_hough_grid(gray_small)
        self._publish_hough_debug(frame_bgr, hough_observation, scale_factor)

        # init / re-init
        prev_feature_count = _count_points(self._prev_pts)
        tracked_points_threshold = self._tracked_points_threshold(prev_feature_count)
        if (
            self._prev_gray is None
            or self._prev_pts is None
            or prev_feature_count < tracked_points_threshold
        ):
            self._prev_gray = gray_small
            self._prev_pts = self._detect_features(gray_small)
            if self._publish_debug_image and self._prev_pts is not None:
                self._publish_debug(frame_bgr, self._prev_pts, scale_factor)
            return

        # LK track
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray_small,
            self._prev_pts,
            None,
            winSize=(self._win_size, self._win_size),
            maxLevel=self._max_level,
            criteria=self._lk_criteria,
        )

        if p1 is None or st is None:
            if self._reinit_if_fail:
                self._prev_gray = gray_small
                self._prev_pts = self._detect_features(gray_small)
            return

        st = st.reshape(-1)
        good_new = p1[st == 1].reshape(-1, 2)
        good_old = self._prev_pts[st == 1].reshape(-1, 2)

        # optional error filter
        if err is not None:
            err = err.reshape(-1)
            good_err = err[st == 1]
            if self._max_track_error > 0:
                mask = good_err <= self._max_track_error
                good_new = good_new[mask]
                good_old = good_old[mask]

        if good_new.shape[0] < tracked_points_threshold:
            self._prev_gray = gray_small
            self._prev_pts = self._detect_features(gray_small)
            if self._publish_debug_image and self._prev_pts is not None:
                self._publish_debug(frame_bgr, self._prev_pts, scale_factor)
            return

        # Estimate affine (partial: rotation + scale + translation)
        M2, inliers = cv2.estimateAffinePartial2D(
            good_old,
            good_new,
            method=cv2.RANSAC,
            ransacReprojThreshold=self._ransac_thresh,
            maxIters=2000,
            confidence=0.99,
            refineIters=10,
        )

        if M2 is None or inliers is None:
            if self._reinit_if_fail:
                self._prev_gray = gray_small
                self._prev_pts = self._detect_features(gray_small)
            return

        inlier_count = int(inliers.sum())
        inlier_threshold = self._inlier_threshold(good_new.shape[0])
        if inlier_count < inlier_threshold:
            self.get_logger().warn(
                'Insufficient inliers for LK transform: '
                f'{inlier_count}/{inlier_threshold}',
                throttle_duration_sec=5.0
            )
            if self._reinit_if_fail:
                self._prev_gray = gray_small
                self._prev_pts = self._detect_features(gray_small)
            return

        # Build raw 3x3 increment in ORIGINAL pixel scale
        m00, m01, tx = M2[0]
        m10, m11, ty = M2[1]
        if scale_factor > 0:
            tx /= scale_factor
            ty /= scale_factor

        H_raw_img = np.array([
            [m00, m01, tx],
            [m10, m11, ty],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        # Center-compensated increment: H_c = T(-c) * H_raw * T(c)
        cx, cy = self._get_rotation_center(w_img, h_img)
        H_comp_img = _T(-cx, -cy) @ H_raw_img @ _T(cx, cy)

        # Map to body frame (scene transform) then invert to recover vehicle motion
        H_scene_raw_body = self._change_of_basis(H_raw_img)
        H_scene_comp_body = self._change_of_basis(H_comp_img)

        try:
            H_motion_raw_body = np.linalg.inv(H_scene_raw_body)
            H_motion_comp_body = np.linalg.inv(H_scene_comp_body)
        except np.linalg.LinAlgError:
            self.get_logger().warn(
                'Failed to invert LK transform; reinitializing tracks.',
                throttle_duration_sec=5.0,
            )
            if self._reinit_if_fail:
                self._prev_gray = gray_small
                self._prev_pts = self._detect_features(gray_small)
            return

        # Accumulate vehicle pose in world frame (world origin is the first frame)
        self._total_raw = self._total_raw @ H_motion_raw_body
        self._total_comp_lk = self._total_comp_lk @ H_motion_comp_body
        self._total_comp = self._total_comp @ H_motion_comp_body
        self._apply_hough_yaw_correction(hough_observation)

        # Publish totals
        self._publish()

        # Update prev state (keep tracking points)
        self._prev_gray = gray_small
        self._prev_pts = good_new.reshape(-1, 1, 2).astype(np.float32)

        if self._publish_debug_image:
            self._publish_debug(frame_bgr, self._prev_pts, scale_factor)

    @staticmethod
    def _build_mapping_matrix(yaw_deg: float, flip_x: bool, flip_y: bool) -> np.ndarray:
        yaw_rad = math.radians(yaw_deg)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        rotation = np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw],
        ], dtype=np.float64)
        flips = np.array([
            [-1.0 if flip_x else 1.0, 0.0],
            [0.0, -1.0 if flip_y else 1.0],
        ], dtype=np.float64)
        return rotation @ flips

    def _change_of_basis(self, transform_image: np.ndarray) -> np.ndarray:
        # Express image-frame transform in the configured body frame:
        # T_body = M * T_image * M^{-1}, where M maps image axes to body axes.
        composed = np.eye(3, dtype=np.float64)
        composed[:2, :2] = self._image_to_body @ transform_image[:2, :2] @ self._body_to_image
        composed[:2, 2] = (self._image_to_body @ transform_image[:2, 2].reshape(2, 1)).flatten()
        return composed


def main(args=None):
    rclpy.init(args=args)
    node = LkTotalTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
