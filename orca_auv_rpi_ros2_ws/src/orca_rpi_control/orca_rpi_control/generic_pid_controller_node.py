import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger

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

        self.declare_parameter('controller_loop_timer_period_s', 1 / 100)
        self._controller_loop_timer_period_s = self.get_parameter('controller_loop_timer_period_s').get_parameter_value().double_value
        self._controller_loop_timer = self.create_timer(self._controller_loop_timer_period_s, self._controller_loop_timer_callback)

        self.declare_parameter('enabled', False)
        self._enabled = bool(self.get_parameter('enabled').value)

        self.declare_parameter('proportional_gain', 0.0)
        self.declare_parameter('integral_gain', 0.0)
        self.declare_parameter('derivative_gain', 0.0)

        self.declare_parameter('derivative_smoothing_factor', 0.5)

        self._reference_input = 0.0
        self._output_feedback = 0.0

        self._integral_controller_integrated_value = 0.0
        self._derivative_controller_previous_stored_value = 0.0

        self._reset_service = self.create_service(
            Trigger,
            f"{self.get_name()}/reset",
            self._on_reset,
        )

        self.add_on_set_parameters_callback(self._on_params)

    def _reference_input_subscription_callback(self, msg):
        self._reference_input = msg.data

    def _output_feedback_subscription_callback(self, msg):
        self._output_feedback = msg.data

    def _on_params(self, params):
        for p in params:
            if p.name == "enabled":
                self._enabled = bool(p.value)
                if not self._enabled:
                    self._reset_state()
                    self._publish_output(0.0)
        return SetParametersResult(successful=True)

    def _on_reset(self, request, response):
        self._reset_state()
        response.success = True
        response.message = "PID controller state reset"
        return response

    def _reset_state(self):
        self._integral_controller_integrated_value = 0.0
        self._derivative_controller_previous_stored_value = 0.0
        self._derivative_controller_new_stored_value = 0.0

    def _publish_output(self, value: float):
        output_msg = Float64()
        output_msg.data = float(value)
        self._manipulated_variable_publisher.publish(output_msg)

    def _controller_loop_timer_callback(self):
        if not self._enabled:
            return

        error = self._reference_input - self._output_feedback

        proportional_gain = self.get_parameter('proportional_gain').get_parameter_value().double_value
        proportional_output = proportional_gain * error

        self._integral_controller_integrated_value += error * self._controller_loop_timer_period_s

        integral_gain = self.get_parameter('integral_gain').get_parameter_value().double_value
        integral_output = integral_gain * self._integral_controller_integrated_value

        derivative_gain = self.get_parameter('derivative_gain').get_parameter_value().double_value
        derivative_smoothing_factor = self.get_parameter('derivative_smoothing_factor').get_parameter_value().double_value
        assert 0.0 <= derivative_smoothing_factor <= 1.0

        self._derivative_controller_new_stored_value = (
            (1 - derivative_smoothing_factor ** self._controller_loop_timer_period_s) * error
            + derivative_smoothing_factor ** self._controller_loop_timer_period_s * self._derivative_controller_previous_stored_value
        )

        derivative_output = derivative_gain * (
            (self._derivative_controller_new_stored_value - self._derivative_controller_previous_stored_value)
            / self._controller_loop_timer_period_s
        )

        self._derivative_controller_previous_stored_value = self._derivative_controller_new_stored_value

        total_output = proportional_output + integral_output + derivative_output

        self._publish_output(total_output)


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
