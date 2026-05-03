import json
import os
import signal
import subprocess

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Wrench
from std_srvs.srv import Trigger
from xy_translation_control_interfaces.action import MoveToPoint

from .backend import protocol
from .backend.aiohttp_server import AIOHTTPServer


class _AsyncParameterClient:
    """Minimal async parameter client for ROS 2 Humble."""

    def __init__(self, node: Node, node_name: str):
        self._client = node.create_client(SetParameters, f"{node_name}/set_parameters")

    def service_is_ready(self) -> bool:
        return self._client.service_is_ready()

    def set_parameters(self, params):
        request = SetParameters.Request()
        request.parameters = [p.to_parameter_msg() for p in params]
        return self._client.call_async(request)


class GUINode(Node):

    def __init__(self):
        super().__init__('gui_node')

        self._thruster_count = 8
        self._initial_pwm_output_signal_value_us = 1500
        self._pwm_output_signal_value_us = [
            self._initial_pwm_output_signal_value_us
            for _ in range(self._thruster_count)
        ]

        self.aiohttp_server = AIOHTTPServer(self._msg_callback)
        self.aiohttp_server.start_threading()
        self._robot_namespace = self.get_namespace().strip('/')

        self._killed_subscription = self.create_subscription(
                msg_type=Bool,
                topic=protocol.TOPIC_KILLED,
                callback=self._killed_callback,
                qos_profile=10
            )
        self._pressure_sensor_depth_subscriber = self.create_subscription(
                msg_type=Float32,
                topic=protocol.TOPIC_DEPTH_M,
                callback=self._pressure_sensor_depth_callback,
                qos_profile=10
            )
        self._stm32_log_subscriber = self.create_subscription(
                msg_type=String,
                topic=protocol.TOPIC_STM32_LOG,
                callback=self._stm32_log_callback,
                qos_profile=10
            )
        self._supervisor_mode_subscriber = self.create_subscription(
                msg_type=String,
                topic=protocol.TOPIC_SYSTEM_MANAGER_MODE,
                callback=self._supervisor_mode_callback,
                qos_profile=10
            )
        self._supervisor_status_subscriber = self.create_subscription(
                msg_type=String,
                topic=protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                callback=self._supervisor_status_callback,
                qos_profile=10
            )
        self._bottom_camera_pid_topic_subscribers = [
            self.create_subscription(
                msg_type=Float64,
                topic=topic,
                callback=lambda msg, topic_name=topic: self._float64_topic_callback(
                    topic_name,
                    msg,
                ),
                qos_profile=10,
            )
            for topic in (
                protocol.TOPIC_BOTTOM_CAMERA_PID_X_REFERENCE_PX,
                protocol.TOPIC_BOTTOM_CAMERA_PID_Y_REFERENCE_PX,
                protocol.TOPIC_BOTTOM_CAMERA_PID_YAW_REFERENCE_RAD,
                protocol.TOPIC_BOTTOM_CAMERA_PID_X_FEEDBACK_PX,
                protocol.TOPIC_BOTTOM_CAMERA_PID_Y_FEEDBACK_PX,
                protocol.TOPIC_BOTTOM_CAMERA_PID_YAW_FEEDBACK_RAD,
            )
        ]

        self._initialize_all_thrusters_client = self.create_client(
            Trigger,
            'thrusters/initialize_all',
        )
        self._flash_stm32_client = self.create_client(Trigger, '/flash_stm32')
        self._supervisor_clients = {
            protocol.SUPERVISOR_SERVICE_SAFE_DISABLED: self.create_client(
                Trigger,
                "system_manager/set_mode/safe_disabled",
            ),
            protocol.SUPERVISOR_SERVICE_MANUAL: self.create_client(
                Trigger,
                "system_manager/set_mode/manual",
            ),
            protocol.SUPERVISOR_SERVICE_DEPTH_HOLD: self.create_client(
                Trigger,
                "system_manager/set_mode/depth_hold",
            ),
            protocol.SUPERVISOR_SERVICE_BOTTOM_CAMERA_HOLD: self.create_client(
                Trigger,
                "system_manager/set_mode/bottom_camera_hold",
            ),
            protocol.SUPERVISOR_SERVICE_RESET_CONTROLLERS: self.create_client(
                Trigger,
                "system_manager/reset_controllers",
            ),
            protocol.SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD: self.create_client(
                Trigger,
                "system_manager/disable/depth_hold",
            ),
            protocol.SUPERVISOR_SERVICE_DISABLE_BOTTOM_CAMERA_HOLD: self.create_client(
                Trigger,
                "system_manager/disable/bottom_camera_hold",
            ),
        }

        self._pwm_output_signal_value_subscription = self.create_subscription(
            msg_type=Int32MultiArray,
            topic=protocol.TOPIC_THRUSTERS_PWM_US,
            callback=self._pwm_output_signal_value_subscription_callback,
            qos_profile=10
        )
        self._thrusters_enabled_subscription = self.create_subscription(
            msg_type=Bool,
            topic=protocol.TOPIC_THRUSTERS_ENABLED,
            callback=self._thrusters_enabled_callback,
            qos_profile=10
        )
        self._electromagnet_set_on_subscription = self.create_subscription(
            msg_type=Bool,
            topic=protocol.TOPIC_ELECTROMAGNET_ENABLED,
            callback=self._electromagnet_set_on_callback,
            qos_profile=10
        )

        self._set_pwm_output_signal_value_publisher = self.create_publisher(
            msg_type=Int32MultiArray,
            topic=protocol.TOPIC_THRUSTERS_PWM_US,
            qos_profile=10
        )
        self._electromagnet_set_on_publisher = self.create_publisher(
            msg_type=Bool,
            topic=protocol.TOPIC_ELECTROMAGNET_ENABLED,
            qos_profile=10
        )

        self._set_output_wrench_at_center_publisher = self.create_publisher(
            Wrench,
            protocol.TOPIC_WRENCH_COMMAND,
            10,
        )
        self._target_depth_publisher = self.create_publisher(
            Float64,
            protocol.TOPIC_TARGET_DEPTH_M,
            10,
        )
        self._target_yaw_publisher = self.create_publisher(
            Float64,
            protocol.TOPIC_BOTTOM_CAMERA_PID_YAW_REFERENCE_RAD,
            10,
        )
        self._move_to_point_action_client = ActionClient(
            self,
            MoveToPoint,
            protocol.MOVE_TO_POINT_ACTION_NAME,
        )
        self._move_to_point_goal_handle = None
        self._process_commands = {
            protocol.PROCESS_BOTTOM_CAMERA_PID_FBC_LAUNCH: [
                "ros2", "launch", "xy_control", "bottom_camera_pid_fbc_launch.py",
                f"namespace:={self._robot_namespace}",
            ],
            protocol.PROCESS_DEPTH_CONTROL_LAUNCH: [
                "ros2", "launch", "depth_control", "depth_control_launch.py",
                f"namespace:={self._robot_namespace}",
            ],
        }
        self._processes = {}

        self._controller_groups = {
            protocol.CONTROLLER_GROUP_BOTTOM_CAMERA_PID_FBC: [
                "x_coordinate_pid_controller_node",
                "y_coordinate_pid_controller_node",
                "yaw_angle_pid_controller_node",
            ],
            protocol.CONTROLLER_GROUP_DEPTH_CONTROL: [
                "depth_pid_controller_node",
            ],
        }
        self._param_clients = {}
        self._controller_supervisor_actions = {
            (
                protocol.CONTROLLER_GROUP_BOTTOM_CAMERA_PID_FBC,
                protocol.CONTROLLER_ACTION_ENABLE,
            ): protocol.SUPERVISOR_SERVICE_BOTTOM_CAMERA_HOLD,
            (
                protocol.CONTROLLER_GROUP_BOTTOM_CAMERA_PID_FBC,
                protocol.CONTROLLER_ACTION_DISABLE,
            ): protocol.SUPERVISOR_SERVICE_DISABLE_BOTTOM_CAMERA_HOLD,
            (
                protocol.CONTROLLER_GROUP_BOTTOM_CAMERA_PID_FBC,
                protocol.CONTROLLER_ACTION_RESET,
            ): protocol.SUPERVISOR_SERVICE_RESET_CONTROLLERS,
            (
                protocol.CONTROLLER_GROUP_DEPTH_CONTROL,
                protocol.CONTROLLER_ACTION_ENABLE,
            ): protocol.SUPERVISOR_SERVICE_DEPTH_HOLD,
            (
                protocol.CONTROLLER_GROUP_DEPTH_CONTROL,
                protocol.CONTROLLER_ACTION_DISABLE,
            ): protocol.SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD,
            (
                protocol.CONTROLLER_GROUP_DEPTH_CONTROL,
                protocol.CONTROLLER_ACTION_RESET,
            ): protocol.SUPERVISOR_SERVICE_RESET_CONTROLLERS,
        }

    def _killed_callback(self, msg):
        self.aiohttp_server.send_topic(protocol.TOPIC_KILLED, msg.data)

    def _pressure_sensor_depth_callback(self, msg: Float32):
        self.aiohttp_server.send_topic(protocol.TOPIC_DEPTH_M, msg.data)

    def _stm32_log_callback(self, msg: String):
        self.aiohttp_server.send_topic(protocol.TOPIC_STM32_LOG, msg.data)

    def _supervisor_mode_callback(self, msg: String):
        self.aiohttp_server.send_topic(protocol.TOPIC_SYSTEM_MANAGER_MODE, msg.data)

    def _supervisor_status_callback(self, msg: String):
        self.aiohttp_server.send_topic(protocol.TOPIC_SYSTEM_MANAGER_STATUS, msg.data)

    def _float64_topic_callback(self, topic_name: str, msg: Float64):
        self.aiohttp_server.send_topic(topic_name, msg.data)

    def _pwm_output_signal_value_subscription_callback(self, msg: Int32MultiArray):
        values = list(msg.data)
        if len(values) < self._thruster_count:
            values += [self._initial_pwm_output_signal_value_us] * (
                self._thruster_count - len(values)
            )
        self._pwm_output_signal_value_us = values[:self._thruster_count]
        self.aiohttp_server.send_topic(
            protocol.TOPIC_THRUSTERS_PWM_US,
            list(self._pwm_output_signal_value_us),
        )

    def _thrusters_enabled_callback(self, msg: Bool):
        self.aiohttp_server.send_topic(protocol.TOPIC_THRUSTERS_ENABLED, msg.data)

    def _electromagnet_set_on_callback(self, msg: Bool):
        self.aiohttp_server.send_topic(protocol.TOPIC_ELECTROMAGNET_ENABLED, msg.data)

    def _msg_callback(self, msg):
        try:
            msg_json_object = json.loads(msg)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Received non-JSON message: {msg}")
            return

        msg_type = msg_json_object.get(protocol.FIELD_TYPE)
        msg_data = msg_json_object.get(protocol.FIELD_DATA, {})

        if msg_type == protocol.TYPE_ACTION:
            action_name = msg_data.get(protocol.FIELD_ACTION_NAME)
            if action_name == protocol.ACTION_INITIALIZE_ALL_THRUSTERS:
                self._initialize_all_thrusters_client.call_async(Trigger.Request())
            elif action_name == protocol.ACTION_FLASH_STM32:
                self._flash_stm32()
            elif action_name == protocol.ACTION_SET_SUPERVISOR_SIMULATION_MODE:
                self._set_supervisor_simulation_mode(bool(msg_data.get("enabled")))
            elif action_name == protocol.ACTION_MOVE_TO_POINT:
                self._send_move_to_point_goal(msg_data)
            elif action_name == protocol.ACTION_CANCEL_MOVE_TO_POINT:
                self._cancel_move_to_point_goal()
            else:
                self.get_logger().warning(f"Unknown action request: {action_name}")

        if msg_type == protocol.TYPE_TOPIC:
            topic_name = msg_data.get(protocol.FIELD_TOPIC_NAME)
            if topic_name == protocol.TOPIC_THRUSTERS_PWM_US:
                try:
                    raw_values = msg_data[protocol.FIELD_MSG]["data"]
                    if not isinstance(raw_values, list):
                        raise TypeError
                    pwm_values = [int(value) for value in raw_values]
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(f"Invalid PWM set message: {msg_json_object}")
                else:
                    if len(pwm_values) < self._thruster_count:
                        pwm_values += [self._initial_pwm_output_signal_value_us] * (
                            self._thruster_count - len(pwm_values)
                        )
                    pwm_values = pwm_values[:self._thruster_count]
                    pwm_array_msg = Int32MultiArray()
                    pwm_array_msg.data = pwm_values
                    self._set_pwm_output_signal_value_publisher.publish(pwm_array_msg)
                    self._pwm_output_signal_value_us = pwm_values

            if topic_name == protocol.TOPIC_WRENCH_COMMAND:
                try:
                    wrench_msg = msg_data[protocol.FIELD_MSG]
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

            if topic_name == protocol.TOPIC_TARGET_DEPTH_M:
                try:
                    target_depth = float(msg_data[protocol.FIELD_MSG]["data"])
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(f"Invalid target depth message: {msg_json_object}")
                else:
                    msg = Float64()
                    msg.data = target_depth
                    self._target_depth_publisher.publish(msg)

            if topic_name == protocol.TOPIC_BOTTOM_CAMERA_PID_YAW_REFERENCE_RAD:
                try:
                    target_yaw = float(msg_data[protocol.FIELD_MSG]["data"])
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(f"Invalid target yaw message: {msg_json_object}")
                else:
                    msg = Float64()
                    msg.data = target_yaw
                    self._target_yaw_publisher.publish(msg)

            if topic_name == protocol.TOPIC_ELECTROMAGNET_ENABLED:
                try:
                    electromagnet_set_on = bool(msg_data[protocol.FIELD_MSG]["data"])
                except (KeyError, TypeError):
                    self.get_logger().warning(
                        f"Invalid electromagnet set message: {msg_json_object}"
                    )
                else:
                    msg = Bool()
                    msg.data = electromagnet_set_on
                    self._electromagnet_set_on_publisher.publish(msg)

        if msg_type == protocol.TYPE_PROCESS:
            target = msg_data.get(protocol.FIELD_TARGET)
            action = msg_data.get(protocol.FIELD_ACTION)
            if not target or target not in self._process_commands:
                self.get_logger().warning(f"Unknown process target: {msg_data}")
            elif action == protocol.PROCESS_ACTION_START:
                self._start_process(target)
            elif action in (protocol.PROCESS_ACTION_STOP, protocol.PROCESS_ACTION_KILL):
                self._stop_process(target)
            else:
                self.get_logger().warning(f"Unknown process action: {msg_data}")

        if msg_type == protocol.TYPE_CONTROLLER:
            group = msg_data.get(protocol.FIELD_GROUP)
            action = msg_data.get(protocol.FIELD_ACTION)
            if not group or group not in self._controller_groups:
                self.get_logger().warning(f"Unknown controller group: {msg_data}")
            elif action == protocol.CONTROLLER_ACTION_SET_PID_PARAMS:
                self._set_group_pid_params(group, msg_data.get(protocol.FIELD_PARAMS, {}))
            elif (group, action) in self._controller_supervisor_actions:
                service_key = self._controller_supervisor_actions[(group, action)]
                self._call_supervisor(service_key)
            else:
                self.get_logger().warning(f"Unknown controller action: {msg_data}")

        if msg_type not in protocol.MESSAGE_TYPES:
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

    def _flash_stm32(self):
        if not self._flash_stm32_client.service_is_ready():
            message = "STM32 flash service not ready."
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_FLASH_STM32_STATUS,
                {"success": False, "message": message},
            )
            return

        future = self._flash_stm32_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_flash_stm32_result)

    def _on_flash_stm32_result(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            message = f"STM32 flash failed: {exc}"
            self.get_logger().error(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_FLASH_STM32_STATUS,
                {"success": False, "message": message},
            )
            return

        self.aiohttp_server.send_topic(
            protocol.TOPIC_FLASH_STM32_STATUS,
            {"success": response.success, "message": response.message},
        )

    def _send_move_to_point_goal(self, data):
        if (
            self._move_to_point_goal_handle is not None
            and self._move_to_point_goal_handle.accepted
        ):
            self._send_move_to_point_status(
                "rejected",
                "A move-to-point goal is already active.",
            )
            return

        try:
            x_px = float(data["x_px"])
            y_px = float(data["y_px"])
            speed_px_s = float(data["speed_px_s"])
        except (KeyError, TypeError, ValueError):
            self._send_move_to_point_status(
                "rejected",
                "Invalid move-to-point goal.",
            )
            return

        if not self._move_to_point_action_client.server_is_ready():
            self._send_move_to_point_status(
                "rejected",
                "Move-to-point action server is not ready.",
            )
            return

        goal_msg = MoveToPoint.Goal()
        goal_msg.x_px = x_px
        goal_msg.y_px = y_px
        goal_msg.speed_px_s = speed_px_s

        self._send_move_to_point_status(
            "sending",
            f"Sending target ({x_px:.3f}, {y_px:.3f}) at {speed_px_s:.3f} px/s.",
            x_px=x_px,
            y_px=y_px,
            speed_px_s=speed_px_s,
        )

        future = self._move_to_point_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_move_to_point_feedback,
        )
        future.add_done_callback(self._on_move_to_point_goal_response)

    def _cancel_move_to_point_goal(self):
        if self._move_to_point_goal_handle is None:
            self._send_move_to_point_status("idle", "No active move-to-point goal.")
            return

        future = self._move_to_point_goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_move_to_point_cancel_response)
        self._send_move_to_point_status("canceling", "Canceling move-to-point goal.")

    def _on_move_to_point_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._move_to_point_goal_handle = None
            self._send_move_to_point_status(
                "failed",
                f"Move-to-point send failed: {exc}",
            )
            return

        if not goal_handle.accepted:
            self._move_to_point_goal_handle = None
            self._send_move_to_point_status(
                "rejected",
                "Move-to-point goal rejected.",
            )
            return

        self._move_to_point_goal_handle = goal_handle
        self._send_move_to_point_status("active", "Move-to-point goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_move_to_point_result)

    def _on_move_to_point_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._send_move_to_point_status(
            "active",
            "Move-to-point goal running.",
            target_x_px=feedback.target_x_px,
            target_y_px=feedback.target_y_px,
            progress=feedback.progress,
            remaining_distance_px=feedback.remaining_distance_px,
        )

    def _on_move_to_point_result(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._move_to_point_goal_handle = None
            self._send_move_to_point_status(
                "failed",
                f"Move-to-point result failed: {exc}",
            )
            return

        result = response.result
        self._move_to_point_goal_handle = None
        status = "succeeded" if result.success else "failed"
        self._send_move_to_point_status(status, result.message)

    def _on_move_to_point_cancel_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._send_move_to_point_status(
                "failed",
                f"Move-to-point cancel failed: {exc}",
            )
            return

        if response.goals_canceling:
            self._send_move_to_point_status(
                "canceling",
                "Move-to-point cancel accepted.",
            )
        else:
            self._send_move_to_point_status(
                "failed",
                "Move-to-point cancel rejected.",
            )

    def _send_move_to_point_status(self, state: str, message: str, **extra):
        self.aiohttp_server.send_topic(
            protocol.TOPIC_MOVE_TO_POINT_STATUS,
            {
                "state": state,
                "message": message,
                **extra,
            },
        )

    def _call_supervisor(self, service_key: str):
        client = self._supervisor_clients.get(service_key)
        if client is None:
            self.get_logger().warning(f"Unknown supervisor service: {service_key}")
            return

        if not client.service_is_ready():
            self.get_logger().warning(f"Supervisor service not ready: {service_key}")
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                f"Supervisor service not ready: {service_key}",
            )
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, key=service_key: self._log_supervisor_result(key, f)
        )

    def _log_supervisor_result(self, service_key: str, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            message = f"Supervisor call failed: {service_key}: {exc}"
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                message,
            )
            return

        message = response.message
        if not response.success:
            self.get_logger().warning(
                f"Supervisor rejected {service_key}: {message}"
            )
        else:
            self.get_logger().info(
                f"Supervisor accepted {service_key}: {message}"
            )
        self.aiohttp_server.send_topic(protocol.TOPIC_SYSTEM_MANAGER_STATUS, message)

    def _set_supervisor_simulation_mode(self, enabled: bool):
        client = self._get_param_client("supervisor_node")
        if not client.service_is_ready():
            message = "Supervisor parameter service not ready"
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                message,
            )
            return

        require_hardware_safety = not enabled
        parameters = [
            Parameter(
                "require_not_killed",
                Parameter.Type.BOOL,
                require_hardware_safety,
            ),
            Parameter(
                "require_thrusters_enabled",
                Parameter.Type.BOOL,
                require_hardware_safety,
            ),
        ]
        future = client.set_parameters(parameters)
        future.add_done_callback(
            lambda f, mode_enabled=enabled: self._log_supervisor_config_result(
                mode_enabled,
                f,
            )
        )

    def _log_supervisor_config_result(self, simulation_mode_enabled: bool, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            message = f"Supervisor simulation mode update failed: {exc}"
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                message,
            )
            return

        for res in response.results:
            if not res.successful:
                message = f"Supervisor simulation mode rejected: {res.reason}"
                self.get_logger().warning(message)
                self.aiohttp_server.send_topic(
                    protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                    message,
                )
                return

        mode = "enabled" if simulation_mode_enabled else "disabled"
        message = f"Supervisor simulation mode {mode}"
        self.get_logger().info(message)
        self.aiohttp_server.send_topic(
            protocol.TOPIC_SYSTEM_MANAGER_STATUS,
            message,
        )

    def _get_param_client(self, node_name: str):
        client = self._param_clients.get(node_name)
        if client is None:
            client = _AsyncParameterClient(self, node_name)
            self._param_clients[node_name] = client
        return client

    def _log_param_result(self, node_name: str, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to set params on {node_name}: {exc}")
            return
        for res in response.results:
            if not res.successful:
                self.get_logger().warning(f"Param set failed on {node_name}: {res.reason}")

    def _set_group_pid_params(self, group: str, params):
        nodes = self._controller_groups.get(group, [])
        if not nodes:
            self.get_logger().warning(f"No nodes configured for {group}")
            return

        try:
            p = float(params["proportional_gain"])
            i = float(params["integral_gain"])
            d = float(params["derivative_gain"])
            smoothing = float(params["derivative_smoothing_factor"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(f"Invalid PID params: {params}")
            return

        if not (0.0 <= smoothing <= 1.0):
            self.get_logger().warning(f"derivative_smoothing_factor out of range: {smoothing}")
            return

        for node_name in nodes:
            client = self._get_param_client(node_name)
            if not client.service_is_ready():
                self.get_logger().warning(f"Parameter service not ready for {node_name}")
                continue
            parameters = [
                Parameter("proportional_gain", Parameter.Type.DOUBLE, p),
                Parameter("integral_gain", Parameter.Type.DOUBLE, i),
                Parameter("derivative_gain", Parameter.Type.DOUBLE, d),
                Parameter("derivative_smoothing_factor", Parameter.Type.DOUBLE, smoothing),
            ]
            future = client.set_parameters(parameters)
            future.add_done_callback(lambda f, n=node_name: self._log_param_result(n, f))

def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
