import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Wrench
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
import cv2
import base64
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .aiohttp_server import AIOHTTPServer
from orca_auv_thruster_interfaces_pkg.action import InitializeThrusterAction

class GUINode(Node):

    def __init__(self):
        super().__init__('gui_node', namespace="orca_auv")

        self.aiohttp_server = AIOHTTPServer(self._msg_callback)
        self.aiohttp_server.start_threading()

        rosout_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self._rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self._rosout_callback,
            rosout_qos
        )

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

        self._set_output_wrench_at_center_publisher = self.create_publisher(Wrench, 'set_output_wrench_at_center_N_Nm', 10)

        self._bridge = CvBridge()
        self._camera_sub = self.create_subscription(
            Image,
            'bottom_camera/image_raw',
            self._camera_callback,
            10
        )

        self._transform_sub = self.create_subscription(
            Float64MultiArray,
            'bottom_camera/total_transform_px',
            self._transform_callback,
            10
        )

    def _rosout_callback(self, msg):
        # Map ROS log levels (DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50)
        log_type = "info"
        if msg.level >= 40:
            log_type = "error"
        elif msg.level >= 30:
            log_type = "warning"
        elif msg.level >= 20:
            # Smart Level Detection: Check keywords in msg.msg
            lowercase_msg = msg.msg.lower()
            if any(k in lowercase_msg for k in ["timeout", "fail", "abort", "retry", "lost", "disconnect", "skipping", "not opened"]):
                log_type = "warning"
            if any(k in lowercase_msg for k in ["error", "critical", "fatal", "failed", "died", "exception"]):
                log_type = "error"
        elif msg.level >= 10:
            log_type = "debug"

        # Use full node name as source
        source_name = msg.name
        
        # self.get_logger().debug(f"Relaying log [{source_name}]")
        
        self.aiohttp_server.send_topic("system_log", {
            "msg": msg.msg, 
            "type": log_type,
            "source": source_name
        })

    def _camera_callback(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            self.aiohttp_server.send_topic("bottom_camera_image", jpg_as_text)
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def _transform_callback(self, msg):
        self.aiohttp_server.send_topic("total_transform_px", list(msg.data))

    def _is_kill_switch_closed_callback(self, msg):
        self.aiohttp_server.send_topic("is_kill_switch_closed", msg.data)

    def _msg_callback(self, msg):
        try:
            msg_json_object = json.loads(msg)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Received non-JSON message: {msg}")
            return

        msg_type = msg_json_object.get("type")
        msg_data = msg_json_object.get("data", {})

        if msg_type == "action":
            self._handle_action_message(msg_data)
        elif msg_type == "topic":
            self._handle_topic_message(msg_data, msg_json_object)
        else:
            self.get_logger().warning(f"Unknown message type: {msg_type}")

    def _handle_action_message(self, msg_data):
        action_name = msg_data.get("action_name")
        if action_name == "initialize_all_thrusters":
            self._initialize_all_thrusters_action_client.send_goal_async(InitializeThrusterAction.Goal())
        else:
            self.get_logger().warning(f"Unknown action request: {action_name}")

    def _handle_topic_message(self, msg_data, msg_json_object):
        topic_name = msg_data.get("topic_name")
        if topic_name == "set_pwm_output_signal_value_us":
            self._handle_pwm_topic(msg_data, msg_json_object)
        elif topic_name == "set_output_wrench_at_center_N_Nm":
            self._handle_wrench_topic(msg_data, msg_json_object)
        else:
            self.get_logger().warning(f"Unknown topic: {topic_name}")

    def _handle_pwm_topic(self, msg_data, msg_json_object):
        try:
            thruster_number = int(msg_data["thruster_number"])
            if 0 <= thruster_number < len(self.__set_pwm_output_signal_value_publishers):
                set_pwm_msg = Int32()
                set_pwm_msg.data = int(msg_data["msg"]["data"])
                self.__set_pwm_output_signal_value_publishers[thruster_number].publish(set_pwm_msg)
            else:
                self.get_logger().warning(f"Invalid thruster number: {thruster_number}")
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(f"Invalid PWM set message: {msg_json_object}")

    def _handle_wrench_topic(self, msg_data, msg_json_object):
        try:
            wrench_msg = msg_data["msg"]
            msg = Wrench()
            msg.force.x = float(wrench_msg["force"]["x"])
            msg.force.y = float(wrench_msg["force"]["y"])
            msg.force.z = float(wrench_msg["force"]["z"])
            msg.torque.x = float(wrench_msg["torque"]["x"])
            msg.torque.y = float(wrench_msg["torque"]["y"])
            msg.torque.z = float(wrench_msg["torque"]["z"])
            self._set_output_wrench_at_center_publisher.publish(msg)
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(f"Invalid wrench message: {msg_json_object}")

def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
