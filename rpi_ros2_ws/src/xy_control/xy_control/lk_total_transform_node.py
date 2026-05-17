#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, Float64MultiArray


def _T(tx: float, ty: float) -> np.ndarray:
    """3x3 translation matrix."""
    return np.array([
        [1.0, 0.0, tx],
        [0.0, 1.0, ty],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


@dataclass
class TileLine:
    pos: float
    angle_offset: float
    weight: float


@dataclass
class TileLineDetection:
    horizontal: List[TileLine]
    vertical: List[TileLine]
    segments: np.ndarray


class LkTotalTransformNode(Node):
    """
    Subscribe:
      - image_topic (sensor_msgs/Image)
      - (optional) camera_info_topic (sensor_msgs/CameraInfo)

    Publish:
      - output_topic (std_msgs/Float64MultiArray): vehicle pose in world frame [x, y, yaw, scale]
      - scalar compensated pose topics for direct PID/controller consumers
      - (optional) output_topic_raw              : raw pose (no center compensation) [x, y, yaw, scale]

    Notes:
      - Detects pool tile grout lines in the bottom camera image using Canny + Hough lines.
      - Lines are grouped into horizontal and vertical tile families. Consecutive frames are
        matched by line offset to estimate the image-space scene motion.
      - The incoming image transform describes how the floor moves in the image; the vehicle
        motion is the inverse of that transform. We accumulate that inverse to get pose in
        the world frame (world origin = first frame).
      - Two running totals:
          raw: accumulated directly
          compensated: H_c = T(-c) * H_raw * T(c) (c = image center or principal point)
        so pure rotations about the chosen center do not introduce spurious translation.
      - Outputs are expressed in the configured body frame (image_to_body mapping + flips).
    """

    def __init__(self):
        super().__init__('lk_total_transform_node')

        # ---- Params (topics) ----
        self.declare_parameter('image_topic', 'camera/bottom/image_raw')
        self.declare_parameter('output_topic', 'camera/bottom/pose_px')
        self.declare_parameter('output_x_topic', 'control/pid/bottom_camera/x/feedback_px')
        self.declare_parameter('output_y_topic', 'control/pid/bottom_camera/y/feedback_px')
        self.declare_parameter('output_yaw_topic', 'state/bottom_camera/yaw_rad')
        self.declare_parameter('output_scale_topic', 'state/bottom_camera/scale')

        # raw output (debug / diagnostics)
        self.declare_parameter('publish_raw_output', True)
        self.declare_parameter('output_topic_raw', 'camera/bottom/pose_raw_px')

        # debug image overlay
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', 'camera/bottom/debug/tile_lines')

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

        # ---- Params (tile line detection) ----
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('canny_threshold1', 60.0)
        self.declare_parameter('canny_threshold2', 160.0)
        self.declare_parameter('hough_threshold', 45)
        self.declare_parameter('min_line_length_px', 45.0)
        self.declare_parameter('max_line_gap_px', 12.0)
        self.declare_parameter('axis_angle_tolerance_deg', 25.0)
        self.declare_parameter('line_merge_distance_px', 8.0)
        self.declare_parameter('min_lines_per_axis', 1)
        self.declare_parameter('max_line_match_distance_px', 35.0)
        self.declare_parameter('min_line_matches', 1)
        self.declare_parameter('reinit_if_fail', True)

        # ---- Load params ----
        self._bridge = CvBridge()
        self._image_topic = self.get_parameter('image_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._output_x_topic = self.get_parameter('output_x_topic').value
        self._output_y_topic = self.get_parameter('output_y_topic').value
        self._output_yaw_topic = self.get_parameter('output_yaw_topic').value
        self._output_scale_topic = self.get_parameter('output_scale_topic').value

        self._publish_raw_output = bool(self.get_parameter('publish_raw_output').value)
        self._output_topic_raw = self.get_parameter('output_topic_raw').value

        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._debug_image_topic = self.get_parameter('debug_image_topic').value

        self._rotation_center_mode = str(self.get_parameter('rotation_center_mode').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        yaw_deg = float(self.get_parameter('image_to_body_yaw_deg').value)
        flip_x = bool(self.get_parameter('flip_image_x').value)
        flip_y = bool(self.get_parameter('flip_image_y').value)
        self._image_to_body = self._build_mapping_matrix(yaw_deg, flip_x, flip_y)
        self._body_to_image = np.linalg.inv(self._image_to_body)

        self._resize_width_px = int(self.get_parameter('resize_width_px').value)
        self._blur_kernel_size = int(self.get_parameter('blur_kernel_size').value)
        self._canny_threshold1 = float(self.get_parameter('canny_threshold1').value)
        self._canny_threshold2 = float(self.get_parameter('canny_threshold2').value)
        self._hough_threshold = int(self.get_parameter('hough_threshold').value)
        self._min_line_length_px = float(self.get_parameter('min_line_length_px').value)
        self._max_line_gap_px = float(self.get_parameter('max_line_gap_px').value)
        self._axis_angle_tolerance = math.radians(
            float(self.get_parameter('axis_angle_tolerance_deg').value)
        )
        self._line_merge_distance_px = float(self.get_parameter('line_merge_distance_px').value)
        self._min_lines_per_axis = int(self.get_parameter('min_lines_per_axis').value)
        self._max_line_match_distance_px = float(self.get_parameter('max_line_match_distance_px').value)
        self._min_line_matches = int(self.get_parameter('min_line_matches').value)
        self._reinit_if_fail = bool(self.get_parameter('reinit_if_fail').value)

        # ---- CameraInfo principal point (optional) ----
        self._have_pp = False
        self._pp_cx = 0.0
        self._pp_cy = 0.0

        # ---- State ----
        self._prev_lines: Optional[TileLineDetection] = None

        # running totals (vehicle pose in world frame)
        self._total_raw = np.eye(3, dtype=np.float64)
        self._total_comp = np.eye(3, dtype=np.float64)

        # ---- Pub/Sub ----
        self._pub_comp = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._pub_x = self.create_publisher(Float64, self._output_x_topic, 10)
        self._pub_y = self.create_publisher(Float64, self._output_y_topic, 10)
        self._pub_yaw = self.create_publisher(Float64, self._output_yaw_topic, 10)
        self._pub_scale = self.create_publisher(Float64, self._output_scale_topic, 10)
        self._pub_raw = None
        if self._publish_raw_output:
            self._pub_raw = self.create_publisher(Float64MultiArray, self._output_topic_raw, 10)

        self._debug_pub = None
        if self._publish_debug_image:
            self._debug_pub = self.create_publisher(Image, self._debug_image_topic, 10)

        self._sub_img = self.create_subscription(Image, self._image_topic, self._on_image, 10)
        self._sub_info = None
        if self._rotation_center_mode == 'principal_point':
            self._sub_info = self.create_subscription(CameraInfo, self._camera_info_topic, self._on_camerainfo, 10)

        self.get_logger().info(
            f'Tile-line total transform node started. image_topic={self._image_topic}, '
            f'output_topic={self._output_topic}, rotation_center_mode={self._rotation_center_mode}, '
            f'publish_raw={self._publish_raw_output}\n'
            f'scalar outputs: x={self._output_x_topic}, y={self._output_y_topic}, '
            f'yaw={self._output_yaw_topic}, scale={self._output_scale_topic}'
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

    @staticmethod
    def _extract_similarity(H: np.ndarray) -> Tuple[float, float, float, float]:
        tx = float(H[0, 2])
        ty = float(H[1, 2])
        a = float(H[0, 0])
        c = float(H[1, 0])
        scale = math.sqrt(a * a + c * c)
        rot = math.atan2(c, a)
        return tx, ty, rot, scale

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

        # raw output (optional diagnostics)
        if self._pub_raw is not None:
            tx2, ty2, rot2, scale2 = self._extract_similarity(self._total_raw)
            msg2 = Float64MultiArray()
            msg2.data = [float(tx2), float(ty2), float(rot2), float(scale2)]
            self._pub_raw.publish(msg2)

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def _detect_tile_lines(self, gray_small: np.ndarray) -> Optional[TileLineDetection]:
        gray_eq = cv2.equalizeHist(gray_small)
        blur_size = self._blur_kernel_size
        if blur_size > 1:
            if blur_size % 2 == 0:
                blur_size += 1
            gray_eq = cv2.GaussianBlur(gray_eq, (blur_size, blur_size), 0)

        edges = cv2.Canny(gray_eq, self._canny_threshold1, self._canny_threshold2)
        lines = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=self._hough_threshold,
            minLineLength=self._min_line_length_px,
            maxLineGap=self._max_line_gap_px,
        )

        if lines is None:
            return None

        h, w = gray_small.shape[:2]
        cx = 0.5 * float(w)
        cy = 0.5 * float(h)
        horizontal_obs = []
        vertical_obs = []
        kept_segments = []

        for line in lines.reshape(-1, 4):
            x1, y1, x2, y2 = [float(v) for v in line]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length <= 0.0:
                continue

            angle = math.atan2(dy, dx) % math.pi
            horizontal_error = min(angle, math.pi - angle)
            vertical_error = abs(angle - 0.5 * math.pi)

            if horizontal_error <= self._axis_angle_tolerance:
                signed_angle = angle if angle <= 0.5 * math.pi else angle - math.pi
                tan_angle = math.tan(signed_angle)
                pos = y1 + tan_angle * (cx - x1)
                horizontal_obs.append((pos, signed_angle, length))
                kept_segments.append(line.astype(np.int32))
            elif vertical_error <= self._axis_angle_tolerance:
                signed_angle = angle - 0.5 * math.pi
                # For a vertical-ish line, solve x at image-center y using dx / dy.
                if abs(dy) < 1e-6:
                    continue
                pos = x1 + (dx / dy) * (cy - y1)
                vertical_obs.append((pos, signed_angle, length))
                kept_segments.append(line.astype(np.int32))

        horizontal = self._cluster_lines(horizontal_obs)
        vertical = self._cluster_lines(vertical_obs)
        if len(horizontal) < self._min_lines_per_axis and len(vertical) < self._min_lines_per_axis:
            return None

        segments = np.array(kept_segments, dtype=np.int32) if kept_segments else np.empty((0, 4), dtype=np.int32)
        return TileLineDetection(horizontal=horizontal, vertical=vertical, segments=segments)

    def _cluster_lines(self, observations) -> List[TileLine]:
        if not observations:
            return []

        observations = sorted(observations, key=lambda item: item[0])
        clusters = []
        current = [observations[0]]
        for obs in observations[1:]:
            if abs(obs[0] - current[-1][0]) <= self._line_merge_distance_px:
                current.append(obs)
            else:
                clusters.append(self._merge_cluster(current))
                current = [obs]
        clusters.append(self._merge_cluster(current))
        return clusters

    @staticmethod
    def _merge_cluster(cluster) -> TileLine:
        weights = np.array([item[2] for item in cluster], dtype=np.float64)
        positions = np.array([item[0] for item in cluster], dtype=np.float64)
        angles = np.array([item[1] for item in cluster], dtype=np.float64)
        total_weight = float(weights.sum())
        if total_weight <= 0.0:
            total_weight = 1.0
            weights = np.ones_like(weights)
        pos = float(np.average(positions, weights=weights))
        angle = float(np.average(angles, weights=weights))
        return TileLine(pos=pos, angle_offset=angle, weight=total_weight)

    def _estimate_scene_motion(
        self,
        prev_lines: TileLineDetection,
        curr_lines: TileLineDetection,
    ) -> Optional[Tuple[float, float, float, int]]:
        dx_values, dx_angles = self._match_line_family(prev_lines.vertical, curr_lines.vertical)
        dy_values, dy_angles = self._match_line_family(prev_lines.horizontal, curr_lines.horizontal)
        match_count = len(dx_values) + len(dy_values)

        if match_count < self._min_line_matches:
            return None

        tx = float(np.median(dx_values)) if dx_values else 0.0
        ty = float(np.median(dy_values)) if dy_values else 0.0
        angle_deltas = dx_angles + dy_angles
        rot = float(np.median(angle_deltas)) if angle_deltas else 0.0
        return tx, ty, rot, match_count

    def _match_line_family(
        self,
        prev_lines: List[TileLine],
        curr_lines: List[TileLine],
    ) -> Tuple[List[float], List[float]]:
        if not prev_lines or not curr_lines:
            return [], []

        curr_used = set()
        pos_deltas = []
        angle_deltas = []
        for prev in prev_lines:
            best_idx = -1
            best_dist = self._max_line_match_distance_px
            for idx, curr in enumerate(curr_lines):
                if idx in curr_used:
                    continue
                dist = abs(curr.pos - prev.pos)
                if dist < best_dist:
                    best_idx = idx
                    best_dist = dist
            if best_idx < 0:
                continue

            curr_used.add(best_idx)
            curr = curr_lines[best_idx]
            pos_deltas.append(curr.pos - prev.pos)
            angle_deltas.append(curr.angle_offset - prev.angle_offset)

        return pos_deltas, angle_deltas

    def _publish_debug(self, frame_bgr: np.ndarray, detection: TileLineDetection, scale_factor: float):
        if self._debug_pub is None or frame_bgr is None or detection is None:
            return

        overlay = frame_bgr.copy()
        if detection.segments.size > 0:
            segments = detection.segments.astype(np.float64)
            if scale_factor > 0.0 and scale_factor != 1.0:
                segments /= scale_factor
            for x1, y1, x2, y2 in segments.astype(np.int32):
                cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)

        try:
            out = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert debug image: {exc}', throttle_duration_sec=5.0)
            return
        self._debug_pub.publish(out)

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
        curr_lines = self._detect_tile_lines(gray_small)

        if curr_lines is None:
            self.get_logger().warn('No tile lines detected in bottom camera image.', throttle_duration_sec=5.0)
            if self._reinit_if_fail:
                self._prev_lines = None
            return

        if self._prev_lines is None:
            self._prev_lines = curr_lines
            if self._publish_debug_image:
                self._publish_debug(frame_bgr, curr_lines, scale_factor)
            return

        motion = self._estimate_scene_motion(self._prev_lines, curr_lines)
        if motion is None:
            self.get_logger().warn('Insufficient matched tile lines for transform.', throttle_duration_sec=5.0)
            if self._reinit_if_fail:
                self._prev_lines = curr_lines
            return

        tx, ty, rot, match_count = motion
        if scale_factor > 0.0:
            tx /= scale_factor
            ty /= scale_factor

        cos_rot = math.cos(rot)
        sin_rot = math.sin(rot)
        H_raw_img = np.array([
            [cos_rot, -sin_rot, tx],
            [sin_rot, cos_rot, ty],
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
            self.get_logger().warn('Failed to invert tile-line transform; reinitializing.', throttle_duration_sec=5.0)
            if self._reinit_if_fail:
                self._prev_lines = curr_lines
            return

        # Accumulate vehicle pose in world frame (world origin is the first frame)
        self._total_raw = self._total_raw @ H_motion_raw_body
        self._total_comp = self._total_comp @ H_motion_comp_body

        self.get_logger().debug(
            f'tile line transform: matches={match_count}, scene_tx={tx:.2f}, scene_ty={ty:.2f}, '
            f'scene_yaw={rot:.4f}'
        )

        # Publish totals
        self._publish()

        # Update prev state
        self._prev_lines = curr_lines

        if self._publish_debug_image:
            self._publish_debug(frame_bgr, curr_lines, scale_factor)

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
