#!/usr/bin/python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Bool
from std_msgs.msg import Int32

from orca_auv_thruster_interfaces_pkg.action import InitializeThrusterAction


class ThrusterControllerNode(Node):

    def __init__(self):
        super().__init__("thruster_controller_node", namespace="orca_auv")

        self.__set_pwm_output_on_publishers = [
            self.create_publisher(
                msg_type=Bool,
                topic=f"thruster_{thruster_number}/set_pwm_output_on",
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

        self.__initialize_thruster_action_servers = [
            ActionServer(
                self,
                InitializeThrusterAction,
                f"thruster_{thruster_number}/initialize_thruster",
                lambda goal_handle, thruster_number=thruster_number: self.__execute_callback_with_thruster_number(goal_handle, thruster_number)
            )
            for thruster_number in range(8)
        ]

    def __execute_callback_with_thruster_number(self, goal_handle, thruster_number):
        self.get_logger().info(f"Execute thruster_{thruster_number}/initialize_thruster action")

        set_pwm_output_on_msg = Bool()
        set_pwm_output_on_msg.data = False
        self.__set_pwm_output_on_publishers[thruster_number].publish(set_pwm_output_on_msg)

        time.sleep(0.5)

        set_pwm_output_signal_value = Int32()
        set_pwm_output_signal_value.data = 1500
        self.__set_pwm_output_signal_value_publishers[thruster_number].publish(set_pwm_output_signal_value)

        time.sleep(0.5)

        set_pwm_output_on_msg = Bool()
        set_pwm_output_on_msg.data = True
        self.__set_pwm_output_on_publishers[thruster_number].publish(set_pwm_output_on_msg)

        time.sleep(2)

        goal_handle.succeed()
        return InitializeThrusterAction.Result()


def main(args=None):
    rclpy.init(args=args)

    thruster_controller_node = ThrusterControllerNode()
    rclpy.spin(thruster_controller_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
