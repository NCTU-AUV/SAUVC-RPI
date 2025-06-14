import rclpy
from rclpy.node import Node
from .aiohttp_server import AIOHTTPServer
import json

from std_msgs.msg import Bool


class GUINode(Node):

    def __init__(self):
        super().__init__('gui_node', namespace="orca_auv")

        self.aiohttp_server = AIOHTTPServer(self._msg_callback)
        self.aiohttp_server.start_threading()

        self._is_kill_switch_closed_subscribers = self.create_subscription(
                msg_type=Bool,
                topic="is_kill_switch_closed",
                callback=self._is_kill_switch_closed_callback,
                qos_profile=10
            )

    def _is_kill_switch_closed_callback(self, msg):
        self.aiohttp_server.send_topic("is_kill_switch_closed", msg.data)

    def _msg_callback(self, msg):
        msg_json_object = json.loads(msg)

        if msg_json_object["type"] == "action":
            print("action")

def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
