#!/usr/bin/python3


import rclpy
from rclpy.node import Node


class ThrusterControllerNode(Node):

    def __init__(self):
        super().__init__("thruster_controller_node", namespace="orca_auv")


def main(args=None):
    rclpy.init(args=args)

    thruster_controller_node = ThrusterControllerNode()
    rclpy.spin(thruster_controller_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
