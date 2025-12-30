#!/usr/bin/python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray

from orca_auv_thruster_interfaces_pkg.action import InitializeThrusterAction


class ThrusterInitializationNode(Node):

    def __init__(self):
        super().__init__("thruster_initialization_node", namespace="orca_auv")

        self._thruster_count = 8
        self._initial_pwm_output_signal_value_us = int(self.declare_parameter("initial_pwm_output_signal_value_us", 1500).value)
        self._pwm_output_signal_value_us = [self._initial_pwm_output_signal_value_us for _ in range(self._thruster_count)]

        self.__set_pwm_output_on_publishers = [
            self.create_publisher(
                msg_type=Bool,
                topic=f"thruster_{thruster_number}/set_pwm_output_on",
                qos_profile=10
            )
            for thruster_number in range(self._thruster_count)
        ]

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

        self.__initialize_thruster_action_servers = [
            ActionServer(
                self,
                InitializeThrusterAction,
                f"thruster_{thruster_number}/initialize_thruster",
                lambda goal_handle, thruster_number=thruster_number: self.__initialize_thruster_action_callback_with_thruster_number(goal_handle, thruster_number)
            )
            for thruster_number in range(self._thruster_count)
        ]

        self.__initialize_all_thrusters_action_servers = ActionServer(
                self,
                InitializeThrusterAction,
                "initialize_all_thrusters",
                self.__initialize_all_thrusters_action_callback
            )

    def __pwm_output_signal_value_subscription_callback(self, msg: Int32MultiArray):
        received_values = list(msg.data)
        if len(received_values) < self._thruster_count:
            received_values += [self._initial_pwm_output_signal_value_us] * (self._thruster_count - len(received_values))
        self._pwm_output_signal_value_us = received_values[:self._thruster_count]

    def __initialize_thruster_action_callback_with_thruster_number(self, goal_handle, thruster_number):
        self.get_logger().info(f"Execute thruster_{thruster_number}/initialize_thruster action")

        set_pwm_output_on_msg = Bool()
        set_pwm_output_on_msg.data = False
        self.__set_pwm_output_on_publishers[thruster_number].publish(set_pwm_output_on_msg)

        time.sleep(0.0)

        pwm_values = list(self._pwm_output_signal_value_us)
        if len(pwm_values) < self._thruster_count:
            pwm_values += [self._initial_pwm_output_signal_value_us] * (self._thruster_count - len(pwm_values))
        pwm_values[thruster_number] = self._initial_pwm_output_signal_value_us
        self.__publish_pwm_output_signal_values(pwm_values)

        time.sleep(0.2)

        set_pwm_output_on_msg = Bool()
        set_pwm_output_on_msg.data = True
        self.__set_pwm_output_on_publishers[thruster_number].publish(set_pwm_output_on_msg)

        time.sleep(0.5)

        goal_handle.succeed()
        return InitializeThrusterAction.Result()

    def __initialize_all_thrusters_action_callback(self, goal_handle):
        self.get_logger().info(f"Execute initialize_all_thrusters action")

        for thruster_number in range(self._thruster_count):
            set_pwm_output_on_msg = Bool()
            set_pwm_output_on_msg.data = False
            self.__set_pwm_output_on_publishers[thruster_number].publish(set_pwm_output_on_msg)

        time.sleep(0.0)

        pwm_values = [self._initial_pwm_output_signal_value_us for _ in range(self._thruster_count)]
        self.__publish_pwm_output_signal_values(pwm_values)

        time.sleep(0.2)

        for thruster_number in range(self._thruster_count):
            set_pwm_output_on_msg = Bool()
            set_pwm_output_on_msg.data = True
            self.__set_pwm_output_on_publishers[thruster_number].publish(set_pwm_output_on_msg)

        time.sleep(0.5)

        goal_handle.succeed()
        return InitializeThrusterAction.Result()

    def __publish_pwm_output_signal_values(self, pwm_values):
        msg = Int32MultiArray()
        msg.data = list(pwm_values[:self._thruster_count])
        self.__set_pwm_output_signal_value_publisher.publish(msg)
        self._pwm_output_signal_value_us = list(msg.data)


def main(args=None):
    rclpy.init(args=args)

    thruster_initialization_node = ThrusterInitializationNode()
    rclpy.spin(thruster_initialization_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
