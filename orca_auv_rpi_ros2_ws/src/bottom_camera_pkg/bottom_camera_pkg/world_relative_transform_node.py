import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WorldRelativeTransformNode(Node):
    def __init__(self):
        super().__init__('bottom_camera_world_relative_transform_node', namespace='orca_auv')

        self.declare_parameter('input_topic', 'bottom_camera/total_transform_px')
        self.declare_parameter('output_topic', 'bottom_camera/total_transform_world')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._transform_sub = self.create_subscription(
            Float64MultiArray,
            input_topic,
            self._transform_callback,
            10
        )
        self._transform_pub = self.create_publisher(Float64MultiArray, output_topic, 10)

    def _transform_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('Incoming transform missing components (expected tx, ty, rotation, scale)')
            return

        tx, ty, rotation, scale = msg.data[:4]

        # Simple inversion: negate translation and rotation, keep scale.
        out_msg = Float64MultiArray()
        out_msg.data = [
            float(-tx),
            float(-ty),
            float(-rotation),
            float(scale),
        ]
        self._transform_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldRelativeTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
