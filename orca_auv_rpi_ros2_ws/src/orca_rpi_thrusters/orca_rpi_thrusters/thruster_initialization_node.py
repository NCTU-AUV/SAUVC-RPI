#!/usr/bin/python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger


class ThrusterInitializationNode(Node):

    def __init__(self):
        super().__init__("thruster_initialization_node", namespace="orca_auv")

        self._thruster_count = 8
        self._initial_pwm_output_signal_value_us = int(self.declare_parameter("initial_pwm_output_signal_value_us", 1500).value)
        self._pwm_output_signal_value_us = [self._initial_pwm_output_signal_value_us for _ in range(self._thruster_count)]
        self._hold_after_enable_duration_s = float(self.declare_parameter("hold_after_enable_duration_s", 3.5).value)

        self.__set_pwm_output_on_publisher = self.create_publisher(
            msg_type=Bool,
            topic="thrusters/set_pwm_output_on",
            qos_profile=10
        )

        self.__is_initializing_publisher = self.create_publisher(
            msg_type=Bool,
            topic="thrusters/is_initializing",
            qos_profile=10
        )

        self.__set_pwm_output_signal_value_publisher = self.create_publisher(
            msg_type=Int32MultiArray,
            topic="thrusters/set_pwm_output_signal_value_us",
            qos_profile=10
        )

        self.__pwm_output_signal_value_subscriber = self.create_subscription(
            msg_type=Int32MultiArray,
            topic="thrusters/set_pwm_output_signal_value_us",
            callback=self.__pwm_output_signal_value_subscription_callback,
            qos_profile=10
        )

        self.__initialize_thruster_services = [
            self.create_service(
                Trigger,
                f"thruster_{thruster_number}/initialize_thruster",
                lambda request, response, thruster_number=thruster_number: self.__initialize_thruster_service_callback_with_thruster_number(request, response, thruster_number)
            )
            for thruster_number in range(self._thruster_count)
        ]

        self.__initialize_all_thrusters_service = self.create_service(
            Trigger,
            "initialize_all_thrusters",
            self.__initialize_all_thrusters_service_callback
        )

    def __pwm_output_signal_value_subscription_callback(self, msg: Int32MultiArray):
        received_values = list(msg.data)
        if len(received_values) < self._thruster_count:
            received_values += [self._initial_pwm_output_signal_value_us] * (self._thruster_count - len(received_values))
        self._pwm_output_signal_value_us = received_values[:self._thruster_count]

    def __initialize_thruster_service_callback_with_thruster_number(self, request, response, thruster_number):
        self.get_logger().info(f"Execute thruster_{thruster_number}/initialize_thruster service")

        self.__publish_is_initializing(True)
        self.__publish_set_pwm_output_on(False)

        time.sleep(0.8)

        pwm_values = list(self._pwm_output_signal_value_us)
        if len(pwm_values) < self._thruster_count:
            pwm_values += [self._initial_pwm_output_signal_value_us] * (self._thruster_count - len(pwm_values))
        pwm_values[thruster_number] = self._initial_pwm_output_signal_value_us
        self.__publish_pwm_output_signal_values(pwm_values)

        time.sleep(0.5)

        self.__publish_set_pwm_output_on(True)

        time.sleep(self._hold_after_enable_duration_s)

        self.__publish_is_initializing(False)

        response.success = True
        response.message = "Thruster initialized"
        return response

    def __initialize_all_thrusters_service_callback(self, request, response):
        self.get_logger().info("Execute initialize_all_thrusters service")

        self.__publish_is_initializing(True)
        self.__publish_set_pwm_output_on(False)

        time.sleep(0.8)

        pwm_values = [self._initial_pwm_output_signal_value_us for _ in range(self._thruster_count)]
        self.__publish_pwm_output_signal_values(pwm_values)

        time.sleep(0.5)

        self.__publish_set_pwm_output_on(True)

        time.sleep(self._hold_after_enable_duration_s)

        self.__publish_is_initializing(False)

        response.success = True
        response.message = "All thrusters initialized"
        return response

    def __publish_pwm_output_signal_values(self, pwm_values):
        msg = Int32MultiArray()
        msg.data = list(pwm_values[:self._thruster_count])
        self.__set_pwm_output_signal_value_publisher.publish(msg)
        self._pwm_output_signal_value_us = list(msg.data)

    def __publish_set_pwm_output_on(self, is_on: bool):
        msg = Bool()
        msg.data = is_on
        self.__set_pwm_output_on_publisher.publish(msg)

    def __publish_is_initializing(self, is_initializing: bool):
        msg = Bool()
        msg.data = is_initializing
        self.__is_initializing_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    thruster_initialization_node = ThrusterInitializationNode()
    rclpy.spin(thruster_initialization_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
