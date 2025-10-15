import rclpy
from rclpy.node import Node
from .aiohttp_server import AIOHTTPServer
import json

from std_msgs.msg import Bool
from std_msgs.msg import Int32

from rclpy.action import ActionClient
from orca_auv_thruster_interfaces_pkg.action import InitializeThrusterAction


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

        self._initialize_all_thrusters_action_client = ActionClient(self, InitializeThrusterAction, '/orca_auv/initialize_all_thrusters')

        self.__set_pwm_output_signal_value_publishers = [
            self.create_publisher(
                msg_type=Int32,
                topic=f"thruster_{thruster_number}/set_pwm_output_signal_value_us",
                qos_profile=10
            )
            for thruster_number in range(8)
        ]

    def _is_kill_switch_closed_callback(self, msg):
        self.aiohttp_server.send_topic("is_kill_switch_closed", msg.data)

    def _msg_callback(self, msg):
        print(msg)
        msg_json_object = json.loads(msg)

        if msg_json_object["type"] == "action":
            if msg_json_object["data"]["action_name"] == "initialize_all_thrusters":
                self._initialize_all_thrusters_action_client.send_goal_async(InitializeThrusterAction.Goal())

        if msg_json_object["type"] == "topic":
            if msg_json_object["data"]["topic_name"] == "set_pwm_output_signal_value_us":
                set_pwm_output_signal_value = Int32()
                set_pwm_output_signal_value.data = int(msg_json_object["data"]["msg"]["data"])
                self.__set_pwm_output_signal_value_publishers[int(msg_json_object["data"]["thruster_number"])].publish(set_pwm_output_signal_value)

def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
