import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64, Int32


class ThrusterForceToPWMOutputSignalNode(Node):

    def __init__(self):
        super().__init__("thruster_force_to_pwm_output_signal_node", namespace="orca_auv")

        self.__set_output_force_subscribers = [
            self.create_subscription(
                msg_type=Float64,
                topic=f"thruster_{thruster_number}/set_output_force_N",
                callback=lambda msg, thruster_number=thruster_number: self.__set_output_force_subscribers_callback(msg, thruster_number),
                qos_profile=10
            )
            for thruster_number in range(8)
        ]

        self.__set_pwm_output_signal_value_publishers = [
            self.create_publisher(
                msg_type=Int32,
                topic=f"thruster_{thruster_number}/set_pwm_output_signal_value_us",
                qos_profile=10
            )
            for thruster_number in range(8)
        ]

    def __get_pwm_output_signal_value_us(self, output_force_N):
        # Using a linear approximation of the technical data from https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
        # Approximation line: pwm_output_signal_value_us = 94.385 * output_force_kg_f + 1500

        STANDARD_GRAVITY_M_PER_S_SQUARED = 9.80665

        output_force_kg_f = output_force_N / STANDARD_GRAVITY_M_PER_S_SQUARED

        pwm_output_signal_value_us = int(94.385 * output_force_kg_f + 1500)

        MAX_PWM_OUTPUT_SIGNAL_VALUE_US = 1500 + 250
        if pwm_output_signal_value_us > MAX_PWM_OUTPUT_SIGNAL_VALUE_US:
            pwm_output_signal_value_us = MAX_PWM_OUTPUT_SIGNAL_VALUE_US

        MIN_PWM_OUTPUT_SIGNAL_VALUE_US = 1500 - 250
        if pwm_output_signal_value_us < MIN_PWM_OUTPUT_SIGNAL_VALUE_US:
            pwm_output_signal_value_us = MIN_PWM_OUTPUT_SIGNAL_VALUE_US

        return pwm_output_signal_value_us

    def __set_output_force_subscribers_callback(self, msg, thruster_number):
        output_force_N = msg.data
        pwm_output_signal_value_us = self.__get_pwm_output_signal_value_us(output_force_N)

        set_pwm_output_signal_value = Int32()
        set_pwm_output_signal_value.data = int(pwm_output_signal_value_us)
        self.__set_pwm_output_signal_value_publishers[thruster_number].publish(set_pwm_output_signal_value)


def main(args=None):
    rclpy.init(args=args)

    thruster_force_to_pwm_output_signal_node = ThrusterForceToPWMOutputSignalNode()
    rclpy.spin(thruster_force_to_pwm_output_signal_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()