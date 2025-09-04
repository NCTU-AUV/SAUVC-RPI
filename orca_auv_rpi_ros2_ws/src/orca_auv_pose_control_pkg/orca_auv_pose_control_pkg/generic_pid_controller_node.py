import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class GenericPIDControllerNode(Node):

    def __init__(self):
        super().__init__('generic_pid_controller_node', namespace="orca_auv")

        self._reference_input_subscriber = self.create_subscription(
            Float64,
            'reference_input',
            self._reference_input_subscription_callback,
            10)

        self._output_feedback_subscriber = self.create_subscription(
            Float64,
            'output_feedback',
            self._output_feedback_subscription_callback,
            10)

        self._manipulated_variable_publisher = self.create_publisher(Float64, 'manipulated_variable', 10)

        self._controller_loop_timer_period_s = 1 / 100
        self._controller_loop_timer = self.create_timer(self._controller_loop_timer_period_s, self._controller_loop_timer_callback)

        self.declare_parameter('proportional_gain', 0.0)
        self.declare_parameter('integral_gain', 0.0)
        self.declare_parameter('derivative_gain', 0.0)

        self.declare_parameter('derivative_smoothing_factor', 0.5)

        self._reference_input = 0.0
        self._output_feedback = 0.0

        self._integral_controller_integrated_value = 0.0
        self._derivative_controller_previous_stored_value = 0.0

    def _reference_input_subscription_callback(self, msg):
        self._reference_input = msg.data

    def _target_depth_subscription_callback(self, msg):
        self._output_feedback = msg.data

    def _controller_loop_timer_callback(self):
        error = self._reference_input - self._output_feedback

        proportional_gain = self.get_parameter('proportional_gain').get_parameter_value().string_double
        proportional_output = proportional_gain * error

        self._integral_controller_integrated_value += error * self._controller_loop_timer_period_s

        integral_gain = self.get_parameter('integral_gain').get_parameter_value().string_double
        integral_output = integral_gain * self._integral_controller_integrated_value

        derivative_gain = self.get_parameter('derivative_gain').get_parameter_value().string_double
        derivative_smoothing_factor = self.get_parameter('derivative_smoothing_factor').get_parameter_value().string_double
        assert 0.0 <= derivative_smoothing_factor <= 1.0

        self._derivative_controller_new_stored_value = (
            (1 - derivative_smoothing_factor) ** self._controller_loop_timer_period_s * error
            + derivative_smoothing_factor ** self._controller_loop_timer_period_s * self._derivative_controller_previous_stored_value
        )

        derivative_output = derivative_gain * (
            (self._derivative_controller_new_stored_value - self._derivative_controller_previous_stored_value)
            / self._controller_loop_timer_period_s
        )

        self._derivative_controller_previous_stored_value = self._derivative_controller_new_stored_value

        total_output = proportional_output + integral_output + derivative_output

        output_msg = Float64()
        output_msg.data = total_output

        self._manipulated_variable_publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)

    generic_pid_controller_node = GenericPIDControllerNode()

    rclpy.spin(generic_pid_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    generic_pid_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
