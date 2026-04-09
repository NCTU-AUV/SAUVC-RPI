import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TotalTransformNode(Node):
    """
    Accumulates frame-to-frame transforms into vehicle pose in the world frame.
    The incoming transform describes how the scene moves in the body frame; we
    invert it to recover vehicle motion and integrate from the first frame.
    """
    def __init__(self):
        super().__init__('bottom_camera_total_transform_node', namespace='orca_auv')

        self.declare_parameter('input_topic', 'bottom_camera/frame_transform_px')
        self.declare_parameter('output_topic', 'bottom_camera/total_transform_world')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._total_matrix = np.eye(3, dtype=np.float64)

        self._transform_sub = self.create_subscription(
            Float64MultiArray,
            input_topic,
            self._transform_callback,
            10
        )
        self._transform_pub = self.create_publisher(Float64MultiArray, output_topic, 10)

    def _transform_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn('Transform message missing components (expected 6 values for 2x3 matrix)')
            return

        m00, m01, tx, m10, m11, ty = msg.data[:6]
        increment_scene = np.array([
            [m00, m01, tx],
            [m10, m11, ty],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        try:
            increment_motion = np.linalg.inv(increment_scene)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Failed to invert frame transform increment; skipping.', throttle_duration_sec=5.0)
            return

        # Accumulate vehicle pose in the world frame (world origin = first frame).
        self._total_matrix = self._total_matrix @ increment_motion

        total_tx, total_ty, total_rotation, total_scale = self._extract_similarity(self._total_matrix)
        out_msg = Float64MultiArray()
        out_msg.data = [float(total_tx), float(total_ty), float(total_rotation), float(total_scale)]
        self._transform_pub.publish(out_msg)

    @staticmethod
    def _extract_similarity(matrix):
        """
        Derive translation, rotation, and isotropic scale from the accumulated affine.
        This discards shear/non-uniform scale in the projection to similarity.
        """
        tx = matrix[0, 2]
        ty = matrix[1, 2]
        a = matrix[0, 0]
        c = matrix[1, 0]
        scale = math.sqrt(a * a + c * c)
        rotation = math.atan2(c, a)
        return tx, ty, rotation, scale


def main(args=None):
    rclpy.init(args=args)
    node = TotalTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
