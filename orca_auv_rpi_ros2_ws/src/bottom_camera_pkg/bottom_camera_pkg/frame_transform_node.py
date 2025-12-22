import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


class FrameTransformNode(Node):

    def __init__(self):
        super().__init__('bottom_camera_frame_transform_node', namespace='orca_auv')

        self.declare_parameter('image_topic', 'bottom_camera/image_raw')
        self.declare_parameter('resize_width_px', 320)
        self.declare_parameter('max_features', 500)
        self.declare_parameter('ransac_reproj_threshold_px', 3.0)
        self.declare_parameter('min_inliers', 15)
        self.declare_parameter('publish_debug_image', False)

        image_topic = self.get_parameter('image_topic').value
        resize_width = int(self.get_parameter('resize_width_px').value)
        max_features = int(self.get_parameter('max_features').value)
        self._ransac_reproj_threshold = float(self.get_parameter('ransac_reproj_threshold_px').value)
        self._min_inliers = int(self.get_parameter('min_inliers').value)
        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        self._bridge = CvBridge()
        self._prev_gray = None
        self._prev_scale_factor = 1.0
        self._orb = cv2.ORB_create(nfeatures=max_features)
        self._resize_width = resize_width

        self._transform_pub = self.create_publisher(Float64MultiArray, 'bottom_camera/frame_transform_px', 10)
        self._image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)
        self._debug_image_pub = self.create_publisher(Image, 'bottom_camera/debug/keypoints', 10)

    def _downscale(self, gray_image):
        if self._resize_width <= 0 or gray_image.shape[1] <= self._resize_width:
            return gray_image, 1.0

        scale = self._resize_width / float(gray_image.shape[1])
        height = int(gray_image.shape[0] * scale)
        resized = cv2.resize(gray_image, (self._resize_width, height), interpolation=cv2.INTER_AREA)
        return resized, scale

    def _image_callback(self, msg: Image):
        frame_bgr = self._to_bgr(msg)
        if frame_bgr is None:
            return

        gray_small, scale_factor = self._prepare_gray(frame_bgr)

        if self._prev_gray is None:
            self._init_reference_frame(gray_small, scale_factor, frame_bgr)
            return

        kp_prev, desc_prev, kp_curr, desc_curr = self._detect_keypoints(gray_small)
        if not self._has_enough_features(kp_prev, kp_curr, desc_prev, desc_curr):
            self._update_reference_frame(gray_small, scale_factor)
            return

        matches = self._match_features(desc_prev, desc_curr)
        if not matches:
            self._update_reference_frame(gray_small, scale_factor)
            return

        matrix, inliers = self._estimate_transform(kp_prev, kp_curr, matches)
        self._update_reference_frame(gray_small, scale_factor)

        if not self._transform_valid(matrix, inliers, kp_curr, frame_bgr, scale_factor):
            return

        self._publish_transform(matrix, scale_factor)
        if self._publish_debug_image:
            inlier_keypoints = [kp_curr[i] for i, flag in enumerate(inliers.flatten()) if flag]
            self._publish_keypoints_debug(frame_bgr, inlier_keypoints, scale_factor)

    def _to_bgr(self, msg: Image):
        try:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            return None

    def _prepare_gray(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        return self._downscale(gray)

    def _init_reference_frame(self, gray_small, scale_factor, frame_bgr):
        self._prev_gray = gray_small
        self._prev_scale_factor = scale_factor
        if self._publish_debug_image:
            self._publish_keypoints_debug(frame_bgr, [], scale_factor)

    def _detect_keypoints(self, gray_small):
        kp_prev, desc_prev = self._orb.detectAndCompute(self._prev_gray, None)
        kp_curr, desc_curr = self._orb.detectAndCompute(gray_small, None)
        return kp_prev, desc_prev, kp_curr, desc_curr

    def _has_enough_features(self, kp_prev, kp_curr, desc_prev, desc_curr):
        if desc_prev is None or desc_curr is None or len(kp_prev) < 4 or len(kp_curr) < 4:
            self.get_logger().warn('Not enough features to estimate transform', throttle_duration_sec=5.0)
            return

        return True

    def _match_features(self, desc_prev, desc_curr):
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = matcher.match(desc_prev, desc_curr)
        if len(matches) < 4:
            self.get_logger().warn('Not enough matches to estimate transform', throttle_duration_sec=5.0)
            return None

        matches = sorted(matches, key=lambda m: m.distance)
        return matches

    def _estimate_transform(self, kp_prev, kp_curr, matches):
        src_pts = np.float32([kp_prev[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_curr[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        matrix, inliers = cv2.estimateAffinePartial2D(
            src_pts,
            dst_pts,
            method=cv2.RANSAC,
            ransacReprojThreshold=self._ransac_reproj_threshold
        )
        return matrix, inliers

    def _update_reference_frame(self, gray_small, scale_factor):
        self._prev_gray = gray_small
        self._prev_scale_factor = scale_factor

    def _transform_valid(self, matrix, inliers, kp_curr, frame_bgr, scale_factor):
        if matrix is None or inliers is None:
            self.get_logger().warn('Failed to estimate transform (matrix is None)', throttle_duration_sec=5.0)
            if self._publish_debug_image:
                self._publish_keypoints_debug(frame_bgr, kp_curr, scale_factor)
            return False

        inlier_count = int(inliers.sum())
        if inlier_count < self._min_inliers:
            self.get_logger().warn(
                f'Insufficient inliers for reliable transform: {inlier_count}',
                throttle_duration_sec=5.0
            )
            if self._publish_debug_image:
                self._publish_keypoints_debug(frame_bgr, kp_curr, scale_factor)
            return False

        return True

    def _publish_transform(self, matrix, scale_factor):
        a, b, tx = matrix[0]
        c, d, ty = matrix[1]
        scale = math.sqrt(a * a + b * b)
        rotation_rad = math.atan2(b, a)

        # Convert translation back to original pixel scale.
        if scale_factor > 0:
            tx /= scale_factor
            ty /= scale_factor

        transform_msg = Float64MultiArray()
        transform_msg.data = [float(tx), float(ty), float(rotation_rad), float(scale)]
        self._transform_pub.publish(transform_msg)

    def destroy_node(self):
        self._prev_gray = None
        super().destroy_node()

    def _publish_keypoints_debug(self, frame_bgr, keypoints, scale_factor):
        # keypoints are on the downscaled frame; scale them back so debug overlay aligns with the original frame.
        if frame_bgr is None:
            return
        if scale_factor > 0 and scale_factor != 1.0:
            scaled_keypoints = [
                cv2.KeyPoint(kp.pt[0] / scale_factor, kp.pt[1] / scale_factor, kp.size / scale_factor, kp.angle,
                             kp.response, kp.octave, kp.class_id)
                for kp in keypoints
            ]
        else:
            scaled_keypoints = keypoints

        overlay = cv2.drawKeypoints(frame_bgr, scaled_keypoints, None, color=(0, 255, 0), flags=cv2.DrawMatchesFlags_DEFAULT)
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert debug image: {exc}', throttle_duration_sec=5.0)
            return
        self._debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrameTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
