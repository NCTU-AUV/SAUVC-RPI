#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from orca_auv_thruster_interfaces_pkg.action import InitializeThrusterAction


class ThrusterControllerNode(Node):

    def __init__(self):
        super().__init__("thruster_controller_node", namespace="orca_auv")

        self.__initialize_thruster_action_servers = [
            ActionServer(
                self,
                InitializeThrusterAction,
                f"thruster_{thruster_number}/initialize_thruster",
                lambda goal_handle: self.__execute_callback_with_thruster_number(goal_handle, thruster_number)
            )
            for thruster_number in range(8)
        ]

    def __execute_callback_with_thruster_number(self, goal_handle, thruster_number):
        pass


def main(args=None):
    rclpy.init(args=args)

    thruster_controller_node = ThrusterControllerNode()
    rclpy.spin(thruster_controller_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
