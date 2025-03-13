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
        return 1500

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