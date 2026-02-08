import json
import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Wrench
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

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
        self._process_commands = {
            "bottom_camera_pid_fbc_launch": ["ros2", "launch", "orca_auv_pose_control_pkg", "bottom_camera_pid_fbc_launch.py"],
            "depth_control_launch": ["ros2", "launch", "orca_auv_pose_control_pkg", "depth_control_launch.py"],
            "waypoint_target_publisher": ["ros2", "run", "orca_auv_pose_control_pkg", "waypoint_target_publisher"],
        }
        self._processes = {}

        self._controller_groups = {
            "bottom_camera_pid_fbc": [
                "/orca_auv/x_coordinate_pid_controller_node",
                "/orca_auv/y_coordinate_pid_controller_node",
            ],
            "depth_control": [
                "/orca_auv/depth_pid_controller_node",
            ],
        }
        self._param_clients = {}
        self._reset_clients = {}

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

        if msg_type == "process":
            target = msg_data.get("target")
            action = msg_data.get("action")
            if not target or target not in self._process_commands:
                self.get_logger().warning(f"Unknown process target: {msg_data}")
            elif action == "start":
                self._start_process(target)
            elif action in ("stop", "kill"):
                self._stop_process(target)
            else:
                self.get_logger().warning(f"Unknown process action: {msg_data}")

        if msg_type == "controller":
            group = msg_data.get("group")
            action = msg_data.get("action")
            if not group or group not in self._controller_groups:
                self.get_logger().warning(f"Unknown controller group: {msg_data}")
            elif action in ("enable", "disable"):
                self._set_group_enabled(group, action == "enable")
            elif action == "reset":
                self._reset_group(group)
            else:
                self.get_logger().warning(f"Unknown controller action: {msg_data}")

        if msg_type not in ("action", "topic", "process", "controller"):
            self.get_logger().warning(f"Unknown message type: {msg_json_object}")

    def destroy_node(self):
        self._stop_all_processes()
        return super().destroy_node()

    def _start_process(self, name: str):
        process = self._processes.get(name)
        if process and process.poll() is None:
            self.get_logger().info(f"{name} already running with pid {process.pid}")
            return

        cmd = self._process_commands.get(name)
        if not cmd:
            self.get_logger().warning(f"No command configured for {name}")
            return

        try:
            preexec_fn = os.setsid if hasattr(os, "setsid") else None
            process = subprocess.Popen(cmd, preexec_fn=preexec_fn)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to start {name}: {exc}")
            return

        self._processes[name] = process
        self.get_logger().info(f"Started {name} (pid {process.pid})")

    def _stop_process(self, name: str):
        process = self._processes.get(name)
        if not process:
            self.get_logger().info(f"No running process tracked for {name}")
            return

        if process.poll() is not None:
            self.get_logger().info(f"{name} already exited with code {process.returncode}")
            self._processes.pop(name, None)
            return

        try:
            if hasattr(os, "killpg"):
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            else:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if hasattr(os, "killpg"):
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                else:
                    process.kill()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to stop {name}: {exc}")
        finally:
            self._processes.pop(name, None)

    def _stop_all_processes(self):
        for name in list(self._processes.keys()):
            self._stop_process(name)

    def _get_param_client(self, node_name: str):
        client = self._param_clients.get(node_name)
        if client is None:
            client = AsyncParameterClient(self, node_name)
            self._param_clients[node_name] = client
        return client

    def _get_reset_client(self, node_name: str):
        client = self._reset_clients.get(node_name)
        if client is None:
            client = self.create_client(Trigger, f"{node_name}/reset")
            self._reset_clients[node_name] = client
        return client

    def _set_group_enabled(self, group: str, enabled: bool):
        nodes = self._controller_groups.get(group, [])
        for node_name in nodes:
            client = self._get_param_client(node_name)
            if not client.service_is_ready():
                self.get_logger().warning(f"Parameter service not ready for {node_name}")
                continue
            param = Parameter("enabled", Parameter.Type.BOOL, enabled)
            future = client.set_parameters([param])
            future.add_done_callback(lambda f, n=node_name: self._log_param_result(n, f))

    def _log_param_result(self, node_name: str, future):
        try:
            results = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to set params on {node_name}: {exc}")
            return
        for res in results:
            if not res.successful:
                self.get_logger().warning(f"Param set failed on {node_name}: {res.reason}")

    def _reset_group(self, group: str):
        nodes = self._controller_groups.get(group, [])
        for node_name in nodes:
            client = self._get_reset_client(node_name)
            if not client.service_is_ready():
                self.get_logger().warning(f"Reset service not ready for {node_name}")
                continue
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda f, n=node_name: self._log_reset_result(n, f))

    def _log_reset_result(self, node_name: str, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Reset call failed on {node_name}: {exc}")
            return
        if not response.success:
            self.get_logger().warning(f"Reset failed on {node_name}: {response.message}")

def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
