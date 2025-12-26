#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


class LkTotalTransformNode(Node):
    """
    Subscribe:
      - bottom_camera/image_raw (sensor_msgs/Image)

    Publish:
      - bottom_camera/total_transform_px (std_msgs/Float64MultiArray)
        data = [total_tx, total_ty, total_rotation_rad, total_scale]

    Notes:
      - 內部用 KLT (Lucas–Kanade / PyrLK) 追蹤特徵點
      - 每幀用 estimateAffinePartial2D(RANSAC) 估 2x3，累積成 total 3x3
      - 輸出格式對齊既有 total_transform_px (tx, ty, rotation, scale)
    """

    def __init__(self):
        super().__init__('bottom_camera_lk_total_transform_node', namespace='orca_auv')

        # ---- Params (topics) ----
        self.declare_parameter('image_topic', 'bottom_camera/image_raw')
        self.declare_parameter('output_topic', 'bottom_camera/total_transform_px')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', 'bottom_camera/debug/lk_tracks')
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
        self.declare_parameter('win_size', 21)          # LK 視窗大小 (win_size x win_size)
        self.declare_parameter('max_level', 2)          # 金字塔層數
        self.declare_parameter('criteria_count', 20)
        self.declare_parameter('criteria_eps', 0.03)
        self.declare_parameter('max_track_error', 50.0) # err 太大就丟掉（可設 0 代表不濾）

        # ---- Params (robust estimation) ----
        self.declare_parameter('ransac_reproj_threshold_px', 3.0)
        self.declare_parameter('min_inliers', 10)
        self.declare_parameter('min_tracked_points', 30)  # 少於這個就重抓點
        self.declare_parameter('reinit_if_fail', True)

        # ---- Load params ----
        self._bridge = CvBridge()

        self._image_topic = self.get_parameter('image_topic').value
        self._output_topic = self.get_parameter('output_topic').value

        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._debug_image_topic = self.get_parameter('debug_image_topic').value

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

        self._ransac_thresh = float(self.get_parameter('ransac_reproj_threshold_px').value)
        self._min_inliers = int(self.get_parameter('min_inliers').value)
        self._min_tracked_points = int(self.get_parameter('min_tracked_points').value)
        self._reinit_if_fail = bool(self.get_parameter('reinit_if_fail').value)

        yaw_deg = float(self.get_parameter('image_to_body_yaw_deg').value)
        flip_x = bool(self.get_parameter('flip_image_x').value)
        flip_y = bool(self.get_parameter('flip_image_y').value)
        self._image_to_body = self._build_mapping_matrix(yaw_deg, flip_x, flip_y)
        self._body_to_image = np.linalg.inv(self._image_to_body)

        # ---- State ----
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None  # (N,1,2) float32 on downscaled image
        self._prev_scale_factor: float = 1.0

        self._total_matrix = np.eye(3, dtype=np.float64)

        # ---- Pub/Sub ----
        self._total_pub = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        if self._publish_debug_image:
            self._debug_pub = self.create_publisher(Image, self._debug_image_topic, 10)
        else:
            self._debug_pub = None

        self._sub = self.create_subscription(Image, self._image_topic, self._on_image, 10)

        self.get_logger().info(
            f'LK total transform node started. image_topic={self._image_topic}, output_topic={self._output_topic}'
        )

    def _to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}', throttle_duration_sec=5.0)
            return None

    def _downscale_gray(self, gray: np.ndarray) -> Tuple[np.ndarray, float]:
        h, w = gray.shape[:2]
        if self._resize_width_px is None or self._resize_width_px <= 0 or w <= self._resize_width_px:
            return gray, 1.0
        scale = float(self._resize_width_px) / float(w)
        new_w = self._resize_width_px
        new_h = max(1, int(round(h * scale)))
        small = cv2.resize(gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
        return small, scale

    def _detect_features(self, gray_small: np.ndarray) -> Optional[np.ndarray]:
        pts = cv2.goodFeaturesToTrack(
            gray_small,
            mask=None,
            maxCorners=self._max_corners,
            qualityLevel=self._quality_level,
            minDistance=self._min_distance,
            blockSize=self._block_size,
        )
        if pts is None:
            return None
        return pts.astype(np.float32)

    @staticmethod
    def _extract_similarity(matrix: np.ndarray) -> Tuple[float, float, float, float]:
        tx = float(matrix[0, 2])
        ty = float(matrix[1, 2])
        a = float(matrix[0, 0])
        c = float(matrix[1, 0])
        scale = math.sqrt(a * a + c * c)
        rotation = math.atan2(c, a)
        return tx, ty, rotation, scale

    def _publish_total(self):
        tx, ty, rot, scale = self._extract_similarity(self._total_matrix)
        msg = Float64MultiArray()
        msg.data = [float(tx), float(ty), float(rot), float(scale)]
        self._total_pub.publish(msg)

    def _publish_debug(self, frame_bgr: np.ndarray, pts_small: np.ndarray, scale_factor: float):
        if self._debug_pub is None:
            return
        if frame_bgr is None or pts_small is None:
            return
        # pts_small: (N,1,2) in downscaled coords, scale back to original
        pts = pts_small.reshape(-1, 2)
        if scale_factor > 0 and scale_factor != 1.0:
            pts = pts / scale_factor

        overlay = frame_bgr.copy()
        for x, y in pts:
            cv2.circle(overlay, (int(x), int(y)), 3, (0, 0, 255), -1)

        try:
            out = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert debug image: {exc}', throttle_duration_sec=5.0)
            return
        self._debug_pub.publish(out)

    def _on_image(self, msg: Image):
        frame_bgr = self._to_bgr(msg)
        if frame_bgr is None:
            return

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray_small, scale_factor = self._downscale_gray(gray)

        # init
        if self._prev_gray is None or self._prev_pts is None or len(self._prev_pts) < self._min_tracked_points:
            self._prev_gray = gray_small
            self._prev_scale_factor = scale_factor
            self._prev_pts = self._detect_features(gray_small)
            if self._publish_debug_image and self._prev_pts is not None:
                self._publish_debug(frame_bgr, self._prev_pts, scale_factor)
            return

        # LK track
        lk_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, self._criteria_count, self._criteria_eps)
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray_small,
            self._prev_pts,
            None,
            winSize=(self._win_size, self._win_size),
            maxLevel=self._max_level,
            criteria=lk_criteria,
        )

        if p1 is None or st is None:
            if self._reinit_if_fail:
                self._prev_gray = gray_small
                self._prev_scale_factor = scale_factor
                self._prev_pts = self._detect_features(gray_small)
            return

        st = st.reshape(-1)
        good_new = p1[st == 1].reshape(-1, 2)
        good_old = self._prev_pts[st == 1].reshape(-1, 2)
        if err is not None:
            err = err.reshape(-1)
            good_err = err[st == 1]
        else:
            good_err = None

        # optional error filter
        if good_err is not None and self._max_track_error > 0:
            mask = good_err <= self._max_track_error
            good_new = good_new[mask]
            good_old = good_old[mask]

        if good_new.shape[0] < self._min_tracked_points:
            # reinit points on current frame
            self._prev_gray = gray_small
            self._prev_scale_factor = scale_factor
            self._prev_pts = self._detect_features(gray_small)
            if self._publish_debug_image and self._prev_pts is not None:
                self._publish_debug(frame_bgr, self._prev_pts, scale_factor)
            return

        # Estimate affine (partial => similarity-ish: rotation+scale+translation, no full shear)
        matrix_2x3, inliers = cv2.estimateAffinePartial2D(
            good_old,
            good_new,
            method=cv2.RANSAC,
            ransacReprojThreshold=self._ransac_thresh,
            maxIters=2000,
            confidence=0.99,
            refineIters=10,
        )

        if matrix_2x3 is None or inliers is None:
            if self._reinit_if_fail:
                self._prev_gray = gray_small
                self._prev_scale_factor = scale_factor
                self._prev_pts = self._detect_features(gray_small)
            return

        inlier_count = int(inliers.sum())
        if inlier_count < self._min_inliers:
            self.get_logger().warn(
                f'Insufficient inliers for LK transform: {inlier_count}',
                throttle_duration_sec=5.0
            )
            if self._reinit_if_fail:
                self._prev_gray = gray_small
                self._prev_scale_factor = scale_factor
                self._prev_pts = self._detect_features(gray_small)
            return

        # Convert translation back to original pixel scale (like your ORB node)
        m00, m01, tx = matrix_2x3[0]
        m10, m11, ty = matrix_2x3[1]
        if scale_factor > 0:
            tx /= scale_factor
            ty /= scale_factor

        increment_image = np.array([
            [m00, m01, tx],
            [m10, m11, ty],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        increment_body = self._change_of_basis(increment_image)

        self._total_matrix = self._total_matrix @ increment_body
        self._publish_total()

        # update prev
        self._prev_gray = gray_small
        self._prev_scale_factor = scale_factor
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
