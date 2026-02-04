#!/usr/bin/python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray

from orca_auv_thruster_interfaces_pkg.action import InitializeThrusterAction


class ThrusterInitializationNode(Node):

    def __init__(self):
        super().__init__("thruster_initialization_node", namespace="orca_auv")

        self._thruster_count = 8
        self._initial_pwm_output_signal_value_us = int(self.declare_parameter("initial_pwm_output_signal_value_us", 1500).value)
        self._pwm_output_signal_value_us = [self._initial_pwm_output_signal_value_us for _ in range(self._thruster_count)]
        self._hold_after_enable_duration_s = float(self.declare_parameter("hold_after_enable_duration_s", 1.0).value)
        self._pwm_output_signal_value_feedback_topic = self.declare_parameter(
            "pwm_output_signal_value_feedback_topic",
            ""
        ).value

        self.__set_pwm_output_on_publisher = self.create_publisher(
            msg_type=Bool,
            topic="thrusters/set_pwm_output_on",
            qos_profile=10
        )

        self.__set_force_to_pwm_output_enabled_publisher = self.create_publisher(
            msg_type=Bool,
            topic="thrusters/set_force_to_pwm_output_enabled",
            qos_profile=10
        )

        self.__set_pwm_output_signal_value_publisher = self.create_publisher(
            msg_type=Int32MultiArray,
            topic="thrusters/set_pwm_output_signal_value_us",
            qos_profile=10
        )

        self.__pwm_output_signal_value_subscriber = None
        if self._pwm_output_signal_value_feedback_topic:
            self.__pwm_output_signal_value_subscriber = self.create_subscription(
                msg_type=Int32MultiArray,
                topic=self._pwm_output_signal_value_feedback_topic,
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

        self.__publish_force_to_pwm_output_enabled(False)
        self.__publish_set_pwm_output_on(False)

        if not self.__sleep_with_cancel_check(goal_handle, 0.0):
            return InitializeThrusterAction.Result()

        pwm_values = list(self._pwm_output_signal_value_us)
        if len(pwm_values) < self._thruster_count:
            pwm_values += [self._initial_pwm_output_signal_value_us] * (self._thruster_count - len(pwm_values))
        pwm_values[thruster_number] = self._initial_pwm_output_signal_value_us
        self.__publish_pwm_output_signal_values(pwm_values)

        if not self.__sleep_with_cancel_check(goal_handle, 0.2):
            return InitializeThrusterAction.Result()

        self.__publish_set_pwm_output_on(True)

        if not self.__sleep_with_cancel_check(goal_handle, self._hold_after_enable_duration_s):
            return InitializeThrusterAction.Result()

        self.__publish_force_to_pwm_output_enabled(True)

        goal_handle.succeed()
        return InitializeThrusterAction.Result()

    def __initialize_all_thrusters_action_callback(self, goal_handle):
        self.get_logger().info(f"Execute initialize_all_thrusters action")

        self.__publish_force_to_pwm_output_enabled(False)
        self.__publish_set_pwm_output_on(False)

        if not self.__sleep_with_cancel_check(goal_handle, 0.0):
            return InitializeThrusterAction.Result()

        pwm_values = [self._initial_pwm_output_signal_value_us for _ in range(self._thruster_count)]
        self.__publish_pwm_output_signal_values(pwm_values)

        if not self.__sleep_with_cancel_check(goal_handle, 0.2):
            return InitializeThrusterAction.Result()

        self.__publish_set_pwm_output_on(True)

        if not self.__sleep_with_cancel_check(goal_handle, self._hold_after_enable_duration_s):
            return InitializeThrusterAction.Result()

        self.__publish_force_to_pwm_output_enabled(True)

        goal_handle.succeed()
        return InitializeThrusterAction.Result()

    def __publish_pwm_output_signal_values(self, pwm_values):
        msg = Int32MultiArray()
        msg.data = list(pwm_values[:self._thruster_count])
        self.__set_pwm_output_signal_value_publisher.publish(msg)
        self._pwm_output_signal_value_us = list(msg.data)

    def __publish_set_pwm_output_on(self, is_on: bool):
        msg = Bool()
        msg.data = is_on
        self.__set_pwm_output_on_publisher.publish(msg)

    def __publish_force_to_pwm_output_enabled(self, is_enabled: bool):
        msg = Bool()
        msg.data = is_enabled
        self.__set_force_to_pwm_output_enabled_publisher.publish(msg)
    
    def __sleep_with_cancel_check(self, goal_handle, duration_s: float) -> bool:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return False
        remaining_s = max(0.0, float(duration_s))
        step_s = 0.05
        while remaining_s > 0.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            sleep_s = min(step_s, remaining_s)
            time.sleep(sleep_s)
            remaining_s -= sleep_s
        return True


def main(args=None):
    rclpy.init(args=args)

    thruster_initialization_node = ThrusterInitializationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(thruster_initialization_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
