import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Float64


class Float32Float64ConverterNode(Node):

    def __init__(self):
        super().__init__('generic_pid_controller_node', namespace="orca_auv")

        self._float32_topic_subscriber = self.create_subscription(
            Float32,
            'float32_topic',
            self._float32_topic_subscription_callback,
            10)

        self._float32_topic_publisher = self.create_publisher(Float32, 'float32_topic', 10)

        self._float64_topic_subscriber = self.create_subscription(
            Float64,
            'float64_topic',
            self._float64_topic_subscription_callback,
            10)

        self._float64_topic_publisher = self.create_publisher(Float64, 'float32_topic', 10)

    def _float32_topic_subscription_callback(self, msg):
        self._float64_topic_publisher.publish(msg.data)

    def _float64_topic_subscription_callback(self, msg):
        self._float32_topic_publisher.publish(msg.data)


def main(args=None):
    rclpy.init(args=args)

    float32_float64_converter_node = Float32Float64ConverterNode()

    rclpy.spin(float32_float64_converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    float32_float64_converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
