import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Wrench
from rclpy.action import ActionClient

from .aiohttp_server import AIOHTTPServer
from orca_auv_thruster_interfaces_pkg.action import InitializeThrusterAction


class GUINode(Node):

    def __init__(self):
        super().__init__('gui_node', namespace="orca_auv")

        self._thruster_count = 8
        self._initial_pwm_output_signal_value_us = 1500
        self._pwm_output_signal_value_us = [self._initial_pwm_output_signal_value_us for _ in range(self._thruster_count)]

        self.aiohttp_server = AIOHTTPServer(self._msg_callback)
        self.aiohttp_server.start_threading()

        self._is_kill_switch_closed_subscribers = self.create_subscription(
                msg_type=Bool,
                topic="is_kill_switch_closed",
                callback=self._is_kill_switch_closed_callback,
                qos_profile=10
            )

        self._initialize_all_thrusters_action_client = ActionClient(self, InitializeThrusterAction, '/orca_auv/initialize_all_thrusters')

        self._pwm_output_signal_value_subscription = self.create_subscription(
            msg_type=Int32MultiArray,
            topic="thrusters/set_pwm_output_signal_value_us",
            callback=self._pwm_output_signal_value_subscription_callback,
            qos_profile=10
        )

        self._set_pwm_output_signal_value_publisher = self.create_publisher(
            msg_type=Int32MultiArray,
            topic="thrusters/set_pwm_output_signal_value_us",
            qos_profile=10
        )

        self._set_output_wrench_at_center_publisher = self.create_publisher(Wrench, 'set_output_wrench_at_center_N_Nm', 10)

    def _is_kill_switch_closed_callback(self, msg):
        self.aiohttp_server.send_topic("is_kill_switch_closed", msg.data)

    def _pwm_output_signal_value_subscription_callback(self, msg: Int32MultiArray):
        values = list(msg.data)
        if len(values) < self._thruster_count:
            values += [self._initial_pwm_output_signal_value_us] * (self._thruster_count - len(values))
        self._pwm_output_signal_value_us = values[:self._thruster_count]

    def _msg_callback(self, msg):
        try:
            msg_json_object = json.loads(msg)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Received non-JSON message: {msg}")
            return

        msg_type = msg_json_object.get("type")
        msg_data = msg_json_object.get("data", {})

        if msg_type == "action":
            action_name = msg_data.get("action_name")
            if action_name == "initialize_all_thrusters":
                self._initialize_all_thrusters_action_client.send_goal_async(InitializeThrusterAction.Goal())
            else:
                self.get_logger().warning(f"Unknown action request: {action_name}")

        if msg_type == "topic":
            topic_name = msg_data.get("topic_name")
            if topic_name == "set_pwm_output_signal_value_us":
                try:
                    thruster_number = int(msg_data["thruster_number"])
                    if thruster_number < 0 or thruster_number >= self._thruster_count:
                        raise IndexError
                    pwm_value = int(msg_data["msg"]["data"])
                except (KeyError, TypeError, ValueError, IndexError):
                    self.get_logger().warning(f"Invalid PWM set message: {msg_json_object}")
                else:
                    pwm_values = list(self._pwm_output_signal_value_us)
                    pwm_values[thruster_number] = pwm_value
                    pwm_array_msg = Int32MultiArray()
                    pwm_array_msg.data = pwm_values
                    self._set_pwm_output_signal_value_publisher.publish(pwm_array_msg)
                    self._pwm_output_signal_value_us = pwm_values

            if topic_name == "set_output_wrench_at_center_N_Nm":
                try:
                    wrench_msg = msg_data["msg"]
                    msg = Wrench()

                    msg.force.x = float(wrench_msg["force"]["x"])
                    msg.force.y = float(wrench_msg["force"]["y"])
                    msg.force.z = float(wrench_msg["force"]["z"])
                    msg.torque.x = float(wrench_msg["torque"]["x"])
                    msg.torque.y = float(wrench_msg["torque"]["y"])
                    msg.torque.z = float(wrench_msg["torque"]["z"])
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(f"Invalid wrench message: {msg_json_object}")
                else:
                    self._set_output_wrench_at_center_publisher.publish(msg)

        if msg_type not in ("action", "topic"):
            self.get_logger().warning(f"Unknown message type: {msg_json_object}")

def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
