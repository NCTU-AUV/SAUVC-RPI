from enum import Enum

import rclpy
from geometry_msgs.msg import Wrench
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_srvs.srv import Trigger


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


class ControlMode(Enum):
    SAFE_DISABLED = "SAFE_DISABLED"
    MANUAL = "MANUAL"
    DEPTH_HOLD = "DEPTH_HOLD"
    BOTTOM_CAMERA_HOLD = "BOTTOM_CAMERA_HOLD"
    DEPTH_AND_BOTTOM_CAMERA_HOLD = "DEPTH_AND_BOTTOM_CAMERA_HOLD"
    FAULT = "FAULT"


class SupervisorNode(Node):
    """Owns high-level control mode and activates controller groups."""

    def __init__(self):
        super().__init__("supervisor_node")

        self.declare_parameter("require_kill_switch_closed", True)
        self.declare_parameter("require_thrusters_enabled", True)
        self.declare_parameter("depth_sensor_timeout_s", 1.0)
        self.declare_parameter("bottom_camera_timeout_s", 1.0)
        self.declare_parameter("publish_zero_wrench_on_disable", True)

        self._controller_groups = {
            "depth_control": [
                "depth_pid_controller_node",
            ],
            "bottom_camera_pid_fbc": [
                "x_coordinate_pid_controller_node",
                "y_coordinate_pid_controller_node",
            ],
        }

        self._mode = ControlMode.SAFE_DISABLED
        self._status = "Initialized in SAFE_DISABLED"
        self._active_controller_groups = set()
        self._kill_switch_closed = False
        self._have_kill_switch = False
        self._thrusters_enabled = False
        self._have_thrusters_enabled = False
        self._last_depth_stamp = None
        self._last_bottom_camera_stamp = None

        self._param_clients = {}
        self._reset_clients = {}

        self._mode_publisher = self.create_publisher(String, "system_manager/mode", 10)
        self._status_publisher = self.create_publisher(String, "system_manager/status", 10)
        self._zero_wrench_publisher = self.create_publisher(Wrench, "control/wrench_command", 10)

        self.create_subscription(Bool, "sensors/kill_switch_closed", self._on_kill_switch, 10)
        self.create_subscription(Bool, "thrusters/enabled", self._on_thrusters_enabled, 10)
        self.create_subscription(Float32, "sensors/depth_m", self._on_depth_float32, 10)
        self.create_subscription(Float64, "state/depth_m", self._on_depth_float64, 10)
        self.create_subscription(
            Float64MultiArray,
            "camera/bottom/pose_px",
            self._on_bottom_camera_pose,
            10,
        )

        self.create_service(
            Trigger,
            "system_manager/set_mode/safe_disabled",
            self._set_safe_disabled,
        )
        self.create_service(Trigger, "system_manager/set_mode/manual", self._set_manual)
        self.create_service(Trigger, "system_manager/set_mode/depth_hold", self._set_depth_hold)
        self.create_service(
            Trigger,
            "system_manager/disable/depth_hold",
            self._disable_depth_hold,
        )
        self.create_service(
            Trigger,
            "system_manager/set_mode/bottom_camera_hold",
            self._set_bottom_camera_hold,
        )
        self.create_service(
            Trigger,
            "system_manager/disable/bottom_camera_hold",
            self._disable_bottom_camera_hold,
        )
        self.create_service(Trigger, "system_manager/reset_controllers", self._reset_controllers)

        self.create_timer(0.2, self._publish_state)
        self.create_timer(0.2, self._check_active_mode_safety)

    def _on_kill_switch(self, msg: Bool):
        self._kill_switch_closed = bool(msg.data)
        self._have_kill_switch = True
        if not self._kill_switch_closed:
            self._enter_fault("Kill switch is open")

    def _on_thrusters_enabled(self, msg: Bool):
        self._thrusters_enabled = bool(msg.data)
        self._have_thrusters_enabled = True
        if not self._thrusters_enabled and self._active_controller_groups:
            self._enter_fault("Thrusters are disabled")

    def _on_depth_float32(self, msg: Float32):
        self._last_depth_stamp = self.get_clock().now()

    def _on_depth_float64(self, msg: Float64):
        self._last_depth_stamp = self.get_clock().now()

    def _on_bottom_camera_pose(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= 2:
            self._last_bottom_camera_stamp = self.get_clock().now()

    def _set_safe_disabled(self, request, response):
        self._set_mode(ControlMode.SAFE_DISABLED, "Operator requested SAFE_DISABLED")
        response.success = True
        response.message = self._status
        return response

    def _set_manual(self, request, response):
        ok, reason = self._safety_ready()
        if not ok:
            self._enter_fault(reason)
            response.success = False
            response.message = reason
            return response

        self._set_mode(ControlMode.MANUAL, "Operator requested MANUAL")
        response.success = True
        response.message = self._status
        return response

    def _set_depth_hold(self, request, response):
        ok, reason = self._safety_ready()
        if ok:
            ok, reason = self._depth_ready()
        if not ok:
            self._enter_fault(reason)
            response.success = False
            response.message = reason
            return response

        self._reset_group("depth_control")
        self._enable_group("depth_control")
        self._active_controller_groups.add("depth_control")
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _disable_depth_hold(self, request, response):
        self._disable_group("depth_control")
        self._active_controller_groups.discard("depth_control")
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _set_bottom_camera_hold(self, request, response):
        ok, reason = self._safety_ready()
        if ok:
            ok, reason = self._bottom_camera_ready()
        if not ok:
            self._enter_fault(reason)
            response.success = False
            response.message = reason
            return response

        self._reset_group("bottom_camera_pid_fbc")
        self._enable_group("bottom_camera_pid_fbc")
        self._active_controller_groups.add("bottom_camera_pid_fbc")
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _disable_bottom_camera_hold(self, request, response):
        self._disable_group("bottom_camera_pid_fbc")
        self._active_controller_groups.discard("bottom_camera_pid_fbc")
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _reset_controllers(self, request, response):
        for group in self._controller_groups:
            self._reset_group(group)
        response.success = True
        response.message = "Controller reset requests sent"
        return response

    def _set_mode(self, mode: ControlMode, status: str):
        self._mode = mode
        self._status = status
        if mode in (ControlMode.SAFE_DISABLED, ControlMode.MANUAL, ControlMode.FAULT):
            self._active_controller_groups.clear()
            self._disable_all_controllers()
        if mode in (ControlMode.SAFE_DISABLED, ControlMode.FAULT):
            self._publish_zero_wrench()

    def _enter_fault(self, reason: str):
        if self._mode == ControlMode.FAULT and self._status == reason:
            return
        self._mode = ControlMode.FAULT
        self._status = reason
        self._active_controller_groups.clear()
        self.get_logger().warning(f"Entering FAULT: {reason}")
        self._disable_all_controllers()
        self._publish_zero_wrench()

    def _refresh_mode_from_active_groups(self):
        depth_active = "depth_control" in self._active_controller_groups
        bottom_camera_active = "bottom_camera_pid_fbc" in self._active_controller_groups

        if depth_active and bottom_camera_active:
            self._mode = ControlMode.DEPTH_AND_BOTTOM_CAMERA_HOLD
            self._status = "Depth hold and bottom camera hold active"
        elif depth_active:
            self._mode = ControlMode.DEPTH_HOLD
            self._status = "Depth hold active"
        elif bottom_camera_active:
            self._mode = ControlMode.BOTTOM_CAMERA_HOLD
            self._status = "Bottom camera hold active"
        else:
            self._mode = ControlMode.SAFE_DISABLED
            self._status = "No controller groups active"
            self._publish_zero_wrench()

    def _safety_ready(self):
        if self.get_parameter("require_kill_switch_closed").value:
            if not self._have_kill_switch:
                return False, "Kill switch state is unknown"
            if not self._kill_switch_closed:
                return False, "Kill switch is open"

        if self.get_parameter("require_thrusters_enabled").value:
            if not self._have_thrusters_enabled:
                return False, "Thruster enabled state is unknown"
            if not self._thrusters_enabled:
                return False, "Thrusters are disabled"

        return True, ""

    def _depth_ready(self):
        timeout_s = float(self.get_parameter("depth_sensor_timeout_s").value)
        if self._last_depth_stamp is None:
            return False, "Depth sensor data has not been received"
        if self._age_s(self._last_depth_stamp) > timeout_s:
            return False, "Depth sensor data is stale"
        return True, ""

    def _bottom_camera_ready(self):
        timeout_s = float(self.get_parameter("bottom_camera_timeout_s").value)
        if self._last_bottom_camera_stamp is None:
            return False, "Bottom camera pose has not been received"
        if self._age_s(self._last_bottom_camera_stamp) > timeout_s:
            return False, "Bottom camera pose is stale"
        return True, ""

    def _check_active_mode_safety(self):
        if not self._active_controller_groups:
            return

        ok, reason = self._safety_ready()
        if not ok:
            self._enter_fault(reason)
            return

        if "depth_control" in self._active_controller_groups:
            ok, reason = self._depth_ready()
            if not ok:
                self._enter_fault(reason)
                return

        if "bottom_camera_pid_fbc" in self._active_controller_groups:
            ok, reason = self._bottom_camera_ready()
            if not ok:
                self._enter_fault(reason)

    def _age_s(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _enable_group(self, group: str):
        self._set_group_enabled(group, True)

    def _disable_group(self, group: str):
        self._set_group_enabled(group, False)

    def _disable_all_controllers(self):
        for group in self._controller_groups:
            self._disable_group(group)

    def _set_group_enabled(self, group: str, enabled: bool):
        for node_name in self._controller_groups.get(group, []):
            client = self._get_param_client(node_name)
            if not client.service_is_ready():
                self.get_logger().warning(f"Parameter service not ready for {node_name}")
                continue
            param = Parameter("enabled", Parameter.Type.BOOL, enabled)
            future = client.set_parameters([param])
            future.add_done_callback(lambda f, n=node_name: self._log_param_result(n, f))

    def _get_param_client(self, node_name: str):
        client = self._param_clients.get(node_name)
        if client is None:
            client = _AsyncParameterClient(self, node_name)
            self._param_clients[node_name] = client
        return client

    def _reset_group(self, group: str):
        for node_name in self._controller_groups.get(group, []):
            client = self._get_reset_client(node_name)
            if not client.service_is_ready():
                self.get_logger().warning(f"Reset service not ready for {node_name}")
                continue
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda f, n=node_name: self._log_reset_result(n, f))

    def _get_reset_client(self, node_name: str):
        client = self._reset_clients.get(node_name)
        if client is None:
            client = self.create_client(Trigger, f"{node_name}/reset")
            self._reset_clients[node_name] = client
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

    def _log_reset_result(self, node_name: str, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Reset call failed on {node_name}: {exc}")
            return
        if not response.success:
            self.get_logger().warning(f"Reset failed on {node_name}: {response.message}")

    def _publish_zero_wrench(self):
        if not self.get_parameter("publish_zero_wrench_on_disable").value:
            return
        msg = Wrench()
        self._zero_wrench_publisher.publish(msg)

    def _publish_state(self):
        mode_msg = String()
        mode_msg.data = self._mode.value
        self._mode_publisher.publish(mode_msg)

        status_msg = String()
        status_msg.data = self._status
        self._status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    supervisor_node = SupervisorNode()
    rclpy.spin(supervisor_node)

    supervisor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
