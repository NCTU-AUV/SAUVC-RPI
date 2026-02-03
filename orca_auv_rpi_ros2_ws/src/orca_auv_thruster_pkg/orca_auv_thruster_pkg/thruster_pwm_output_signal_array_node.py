import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray


class ThrusterPWMOutputSignalArrayNode(Node):

    def __init__(self):
        super().__init__("thruster_pwm_output_signal_array_node", namespace="orca_auv")

        self._thruster_count = 8
        initial_pwm_us = int(self.declare_parameter("initial_pwm_output_signal_value_us", 1500).value)
        self._pwm_output_signal_value_us = [initial_pwm_us for _ in range(self._thruster_count)]

        self._publisher = self.create_publisher(
            msg_type=Int32MultiArray,
            topic="thrusters/set_pwm_output_signal_value_us",
            qos_profile=10
        )

        self._subscriptions = [
            self.create_subscription(
                msg_type=Int32,
                topic=f"thruster_{thruster_number}/set_pwm_output_signal_value_us",
                callback=lambda msg, thruster_number=thruster_number: self._set_pwm_output_signal_value_callback(msg, thruster_number),
                qos_profile=10
            )
            for thruster_number in range(self._thruster_count)
        ]

    def _set_pwm_output_signal_value_callback(self, msg, thruster_number):
        self._pwm_output_signal_value_us[thruster_number] = msg.data
        self._publish_pwm_array()

    def _publish_pwm_array(self):
        pwm_array_msg = Int32MultiArray()
        pwm_array_msg.data = list(self._pwm_output_signal_value_us)
        self._publisher.publish(pwm_array_msg)


def main(args=None):
    rclpy.init(args=args)

    thruster_pwm_output_signal_array_node = ThrusterPWMOutputSignalArrayNode()
    rclpy.spin(thruster_pwm_output_signal_array_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
