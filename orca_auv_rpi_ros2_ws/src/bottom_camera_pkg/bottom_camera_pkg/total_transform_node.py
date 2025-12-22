import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TotalTransformNode(Node):
    def __init__(self):
        super().__init__('bottom_camera_total_transform_node', namespace='orca_auv')

        self.declare_parameter('input_topic', 'bottom_camera/frame_transform_px')
        self.declare_parameter('output_topic', 'bottom_camera/total_transform_px')

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
        if len(msg.data) < 4:
            self.get_logger().warn('Transform message missing components (expected tx, ty, rotation, scale)')
            return

        tx, ty, rotation, scale = msg.data[:4]
        increment = self._matrix_from_transform(tx, ty, rotation, scale)
        self._total_matrix = self._total_matrix @ increment

        total_tx, total_ty, total_rotation, total_scale = self._transform_from_matrix(self._total_matrix)
        out_msg = Float64MultiArray()
        out_msg.data = [float(total_tx), float(total_ty), float(total_rotation), float(total_scale)]
        self._transform_pub.publish(out_msg)

    @staticmethod
    def _matrix_from_transform(tx, ty, rotation, scale):
        cos_r = math.cos(rotation)
        sin_r = math.sin(rotation)
        return np.array([
            [scale * cos_r, -scale * sin_r, tx],
            [scale * sin_r,  scale * cos_r, ty],
            [0.0,            0.0,           1.0],
        ], dtype=np.float64)

    @staticmethod
    def _transform_from_matrix(matrix):
        a, b, tx = matrix[0]
        c, d, ty = matrix[1]
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
