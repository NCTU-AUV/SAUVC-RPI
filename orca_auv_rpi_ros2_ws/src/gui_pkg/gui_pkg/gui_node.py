import rclpy
from rclpy.node import Node
from aiohttp_server import AIOHTTPServer


class GUINode(Node):

    def __init__(self):
        super().__init__('gui_node', namespace="orca_auv")

        self.aiohttp_server = AIOHTTPServer(lambda msg: print(msg))
        self.aiohttp_server.start_threading()


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
