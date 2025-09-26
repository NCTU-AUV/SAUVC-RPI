import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench


class OutputSinkForceToOutputWrenchNode(Node):

    def __init__(self):
        super().__init__('output_sink_force_to_output_wrench_node', namespace="orca_auv")

        self._output_sink_force_subscriber = self.create_subscription(
            Float64,
            'output_sink_force_N',
            self._output_sink_force_subscription_callback,
            10)

        self._set_output_wrench_at_center_publisher = self.create_publisher(Wrench, 'set_output_wrench_at_center_N_Nm', 10)

    def _output_sink_force_subscription_callback(self, msg):
        output_sink_force_N = msg.data

        msg = Wrench()

        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = output_sink_force_N
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0

        self._set_output_wrench_at_center_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    output_sink_force_to_output_wrench_node = OutputSinkForceToOutputWrenchNode()

    rclpy.spin(output_sink_force_to_output_wrench_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    output_sink_force_to_output_wrench_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
