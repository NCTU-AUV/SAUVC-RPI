#!/usr/bin/env python3
import math
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from xy_translation_control_interfaces.action import MoveToPoint


class MissionState(Enum):
    WAIT_SYSTEM = auto()
    CALL_DEPTH_HOLD = auto()
    WAIT_TRIGGER_RESULT = auto()
    WAIT_REACH_DEPTH = auto()
    CALL_BOTTOM_CAMERA_HOLD = auto()
    WAIT_MOVE_SERVER = auto()
    SEND_MOVE_GOAL = auto()
    WAIT_GOAL_ACCEPT = auto()
    MOVING = auto()
    DONE = auto()
    FAILED = auto()


class DiveThenForwardMissionNode(Node):
    """Run a dive-then-forward mission."""

    def __init__(self):
        super().__init__("dive_then_forward_mission_node")

        # =========================
        # Parameters
        # =========================

        # Target depth in meters.
        self.declare_parameter("target_depth_m", 0.3)

        # Allowed depth error before we consider the AUV to have reached target depth.
        self.declare_parameter("depth_tolerance_m", 0.03)

        # Depth must stay within tolerance for this long before moving forward.
        self.declare_parameter("depth_stable_time_s", 2.0)

        # Forward direction in your coordinate system is +Y.
        # Therefore target point is x = 0, y = +2000.
        self.declare_parameter("target_x_px", 0.0)
        self.declare_parameter("target_y_px", 2000.0)

        # Keep facing forward.
        # In this project, yaw target is in radians.
        # 0.0 rad means keep the forward heading defined by the bottom-camera frame.
        self.declare_parameter("target_yaw_rad", 0.0)

        # Only used for warning logs. The controller itself still tries to correct yaw.
        self.declare_parameter("yaw_tolerance_rad", 0.10)

        # Speed of waypoint setpoint generation in px/s.
        self.declare_parameter("speed_px_s", 50.0)

        # Mission loop period.
        self.declare_parameter("control_period_s", 0.1)

        # Timeout for waiting to reach depth. Simulation can dive slowly.
        self.declare_parameter("max_depth_wait_s", 180.0)

        # Timeout for movement action.
        self.declare_parameter("max_move_wait_s", 120.0)

        self.target_depth_m = float(self.get_parameter("target_depth_m").value)
        self.depth_tolerance_m = float(self.get_parameter("depth_tolerance_m").value)
        self.depth_stable_time_s = float(self.get_parameter("depth_stable_time_s").value)

        self.target_x_px = float(self.get_parameter("target_x_px").value)
        self.target_y_px = float(self.get_parameter("target_y_px").value)

        self.target_yaw_rad = float(self.get_parameter("target_yaw_rad").value)
        self.yaw_tolerance_rad = float(self.get_parameter("yaw_tolerance_rad").value)

        self.speed_px_s = float(self.get_parameter("speed_px_s").value)

        self.control_period_s = float(self.get_parameter("control_period_s").value)
        self.max_depth_wait_s = float(self.get_parameter("max_depth_wait_s").value)
        self.max_move_wait_s = float(self.get_parameter("max_move_wait_s").value)

        # =========================
        # Publishers / Subscribers
        # =========================

        # Depth target for depth PID.
        self.depth_target_pub = self.create_publisher(
            Float64,
            "control/targets/depth_m",
            10,
        )

        # Yaw target for bottom-camera yaw PID.
        self.yaw_target_pub = self.create_publisher(
            Float64,
            "control/targets/bottom_camera/yaw_rad",
            10,
        )

        # Current depth feedback.
        self.depth_sub = self.create_subscription(
            Float64,
            "state/depth_m",
            self._on_depth,
            10,
        )

        # Current yaw feedback from bottom-camera pose estimation.
        self.yaw_feedback_sub = self.create_subscription(
            Float64,
            "state/bottom_camera/yaw_rad",
            self._on_yaw_feedback,
            10,
        )

        # Optional XY feedback, only for logging / checking.
        self.x_feedback_sub = self.create_subscription(
            Float64,
            "control/pid/bottom_camera/x/feedback_px",
            self._on_x_feedback,
            10,
        )

        self.y_feedback_sub = self.create_subscription(
            Float64,
            "control/pid/bottom_camera/y/feedback_px",
            self._on_y_feedback,
            10,
        )

        # =========================
        # Supervisor services
        # =========================

        self.depth_hold_client = self.create_client(
            Trigger,
            "system_manager/set_mode/depth_hold",
        )

        self.bottom_camera_hold_client = self.create_client(
            Trigger,
            "system_manager/set_mode/bottom_camera_hold",
        )

        # =========================
        # MoveToPoint action client
        # =========================

        self.move_client = ActionClient(
            self,
            MoveToPoint,
            "control/targets/move_to_point",
        )

        # =========================
        # Mission runtime variables
        # =========================

        self.state = MissionState.WAIT_SYSTEM

        self.current_depth_m = None
        self.current_yaw_rad = None
        self.current_x_px = None
        self.current_y_px = None

        self.depth_reached_since_s = None

        self.pending_trigger_future = None
        self.pending_trigger_name = ""
        self.pending_trigger_next_state = None

        self.goal_handle = None
        self.move_result_future = None

        self.state_start_time_s = self._now_s()

        self.timer = self.create_timer(self.control_period_s, self._tick)

        self.get_logger().info(
            "Mission node started. "
            f"Target depth = {self.target_depth_m:.3f} m, "
            f"target point = ({self.target_x_px:.1f}, {self.target_y_px:.1f}) px, "
            f"target yaw = {self.target_yaw_rad:.3f} rad, "
            f"speed = {self.speed_px_s:.1f} px/s"
        )

    # =========================
    # Subscriber callbacks
    # =========================

    def _on_depth(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_depth_m = float(msg.data)

    def _on_yaw_feedback(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_yaw_rad = float(msg.data)

    def _on_x_feedback(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_x_px = float(msg.data)

    def _on_y_feedback(self, msg: Float64):
        if math.isfinite(msg.data):
            self.current_y_px = float(msg.data)

    # =========================
    # Main mission loop
    # =========================

    def _tick(self):
        # Keep depth and yaw targets alive during the whole active mission.
        # Depth target keeps the AUV at 0.3 m.
        # Yaw target keeps the AUV facing forward.
        if self.state not in (MissionState.DONE, MissionState.FAILED):
            self._publish_depth_target()
            self._publish_yaw_target()

        if self.state == MissionState.WAIT_SYSTEM:
            self._tick_wait_system()

        elif self.state == MissionState.CALL_DEPTH_HOLD:
            self._call_trigger_service(
                client=self.depth_hold_client,
                service_name="depth_hold",
                next_state=MissionState.WAIT_REACH_DEPTH,
            )

        elif self.state == MissionState.WAIT_TRIGGER_RESULT:
            self._tick_wait_trigger_result()

        elif self.state == MissionState.WAIT_REACH_DEPTH:
            self._tick_wait_reach_depth()

        elif self.state == MissionState.CALL_BOTTOM_CAMERA_HOLD:
            self._call_trigger_service(
                client=self.bottom_camera_hold_client,
                service_name="bottom_camera_hold",
                next_state=MissionState.WAIT_MOVE_SERVER,
            )

        elif self.state == MissionState.WAIT_MOVE_SERVER:
            self._tick_wait_move_server()

        elif self.state == MissionState.SEND_MOVE_GOAL:
            self._send_move_goal()

        elif self.state == MissionState.WAIT_GOAL_ACCEPT:
            # Result is handled in _on_goal_response callback.
            pass

        elif self.state == MissionState.MOVING:
            self._tick_moving()

        elif self.state == MissionState.DONE:
            # Mission finished successfully.
            pass

        elif self.state == MissionState.FAILED:
            # Mission failed. Keep node alive for debugging.
            pass

    # =========================
    # State handlers
    # =========================

    def _tick_wait_system(self):
        if self.current_depth_m is None:
            self.get_logger().info(
                "Waiting for state/depth_m...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.depth_hold_client.service_is_ready():
            self.get_logger().info(
                "Waiting for system_manager/set_mode/depth_hold service...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.bottom_camera_hold_client.service_is_ready():
            self.get_logger().info(
                "Waiting for system_manager/set_mode/bottom_camera_hold service...",
                throttle_duration_sec=2.0,
            )
            return

        self._set_state(MissionState.CALL_DEPTH_HOLD)

    def _tick_wait_reach_depth(self):
        if self.current_depth_m is None:
            self.get_logger().info(
                "Waiting for depth feedback...",
                throttle_duration_sec=2.0,
            )
            return

        if self._time_in_state_s() > self.max_depth_wait_s:
            self._fail(
                "Timeout while waiting to reach depth. "
                f"current_depth = {self.current_depth_m:.3f}, "
                f"target_depth = {self.target_depth_m:.3f}"
            )
            return

        error = abs(self.current_depth_m - self.target_depth_m)

        if error <= self.depth_tolerance_m:
            if self.depth_reached_since_s is None:
                self.depth_reached_since_s = self._now_s()

            stable_time_s = self._now_s() - self.depth_reached_since_s

            self.get_logger().info(
                "Depth is within tolerance. "
                f"current = {self.current_depth_m:.3f}, "
                f"target = {self.target_depth_m:.3f}, "
                f"error = {error:.3f}, "
                f"stable_time = {stable_time_s:.1f}s",
                throttle_duration_sec=1.0,
            )

            if stable_time_s >= self.depth_stable_time_s:
                self._set_state(MissionState.CALL_BOTTOM_CAMERA_HOLD)

        else:
            self.depth_reached_since_s = None

            self.get_logger().info(
                "Waiting to reach target depth. "
                f"current = {self.current_depth_m:.3f}, "
                f"target = {self.target_depth_m:.3f}, "
                f"error = {error:.3f}",
                throttle_duration_sec=1.0,
            )

    def _tick_wait_move_server(self):
        if not self.move_client.server_is_ready():
            self.get_logger().info(
                "Waiting for MoveToPoint action server...",
                throttle_duration_sec=2.0,
            )
            return

        self._set_state(MissionState.SEND_MOVE_GOAL)

    def _tick_moving(self):
        if self._time_in_state_s() > self.max_move_wait_s:
            self._fail("Timeout during MoveToPoint.")
            return

        # Depth warning. Depth PID should still be correcting it.
        if self.current_depth_m is not None:
            depth_error = abs(self.current_depth_m - self.target_depth_m)

            if depth_error > self.depth_tolerance_m * 3.0:
                self.get_logger().warn(
                    "Depth deviation during movement. "
                    f"current = {self.current_depth_m:.3f}, "
                    f"target = {self.target_depth_m:.3f}, "
                    f"error = {depth_error:.3f}",
                    throttle_duration_sec=1.0,
                )

        # Yaw warning. Yaw PID should still be correcting it.
        if self.current_yaw_rad is not None:
            yaw_error = self._angle_error(self.target_yaw_rad, self.current_yaw_rad)

            if abs(yaw_error) > self.yaw_tolerance_rad:
                self.get_logger().warn(
                    "Yaw deviation during movement. "
                    f"current = {self.current_yaw_rad:.3f}, "
                    f"target = {self.target_yaw_rad:.3f}, "
                    f"error = {yaw_error:.3f}",
                    throttle_duration_sec=1.0,
                )

        if self.current_x_px is not None and self.current_y_px is not None:
            yaw = self.current_yaw_rad
            yaw_text = f"{yaw:.3f}" if yaw is not None else "nan"
            self.get_logger().info(
                "Current bottom-camera feedback: "
                f"x = {self.current_x_px:.1f}, "
                f"y = {self.current_y_px:.1f}, "
                f"yaw = {yaw_text}",
                throttle_duration_sec=2.0,
            )

    # =========================
    # Service handling
    # =========================

    def _call_trigger_service(self, client, service_name: str, next_state: MissionState):
        if not client.service_is_ready():
            self.get_logger().info(
                f"Waiting for {service_name} service...",
                throttle_duration_sec=2.0,
            )
            return

        self.get_logger().info(f"Calling supervisor service: {service_name}")

        request = Trigger.Request()
        self.pending_trigger_future = client.call_async(request)
        self.pending_trigger_name = service_name
        self.pending_trigger_next_state = next_state

        self._set_state(MissionState.WAIT_TRIGGER_RESULT)

    def _tick_wait_trigger_result(self):
        if self.pending_trigger_future is None:
            self._fail("Internal error: no pending trigger service call.")
            return

        if not self.pending_trigger_future.done():
            return

        try:
            response = self.pending_trigger_future.result()
        except Exception as exc:
            self._fail(f"{self.pending_trigger_name} service call failed: {exc}")
            return

        if not response.success:
            self._fail(
                f"{self.pending_trigger_name} rejected. "
                f"message = {response.message}"
            )
            return

        self.get_logger().info(
            f"{self.pending_trigger_name} succeeded. "
            f"message = {response.message}"
        )

        next_state = self.pending_trigger_next_state

        self.pending_trigger_future = None
        self.pending_trigger_name = ""
        self.pending_trigger_next_state = None

        self._set_state(next_state)

    # =========================
    # Action handling
    # =========================

    def _send_move_goal(self):
        goal = MoveToPoint.Goal()

        # Your coordinate definition:
        # forward = +Y
        # target = x 0, y +2000.
        goal.x_px = self.target_x_px
        goal.y_px = self.target_y_px
        goal.speed_px_s = self.speed_px_s

        self.get_logger().info(
            "Sending MoveToPoint goal: "
            f"x = {goal.x_px:.1f}, "
            f"y = {goal.y_px:.1f}, "
            f"speed = {goal.speed_px_s:.1f}, "
            f"yaw hold = {self.target_yaw_rad:.3f} rad"
        )

        send_future = self.move_client.send_goal_async(
            goal,
            feedback_callback=self._on_move_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

        self._set_state(MissionState.WAIT_GOAL_ACCEPT)

    def _on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self._fail(f"Failed to send MoveToPoint goal: {exc}")
            return

        if not self.goal_handle.accepted:
            self._fail("MoveToPoint goal was rejected.")
            return

        self.get_logger().info("MoveToPoint goal accepted.")

        self.move_result_future = self.goal_handle.get_result_async()
        self.move_result_future.add_done_callback(self._on_move_result)

        self._set_state(MissionState.MOVING)

    def _on_move_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            "MoveToPoint feedback: "
            f"target_x = {feedback.target_x_px:.1f}, "
            f"target_y = {feedback.target_y_px:.1f}, "
            f"progress = {feedback.progress * 100.0:.1f}%, "
            f"remaining = {feedback.remaining_distance_px:.1f} px",
            throttle_duration_sec=1.0,
        )

    def _on_move_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._fail(f"Failed to get MoveToPoint result: {exc}")
            return

        if result.success:
            self.get_logger().info(f"Mission completed. message = {result.message}")
            self._set_state(MissionState.DONE)
        else:
            self._fail(f"MoveToPoint failed. message = {result.message}")

    # =========================
    # Target publishers
    # =========================

    def _publish_depth_target(self):
        msg = Float64()
        msg.data = self.target_depth_m
        self.depth_target_pub.publish(msg)

    def _publish_yaw_target(self):
        msg = Float64()
        msg.data = self.target_yaw_rad
        self.yaw_target_pub.publish(msg)

    # =========================
    # Utility functions
    # =========================

    def _set_state(self, new_state: MissionState):
        if self.state == new_state:
            return

        self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")

        self.state = new_state
        self.state_start_time_s = self._now_s()

        if new_state == MissionState.WAIT_REACH_DEPTH:
            self.depth_reached_since_s = None

    def _fail(self, reason: str):
        self.get_logger().error(f"Mission failed: {reason}")
        self._set_state(MissionState.FAILED)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _time_in_state_s(self) -> float:
        return self._now_s() - self.state_start_time_s

    @staticmethod
    def _angle_error(target: float, current: float) -> float:
        """Return shortest angular error target - current in range [-pi, pi]."""
        error = target - current
        return math.atan2(math.sin(error), math.cos(error))


def main(args=None):
    rclpy.init(args=args)

    node = DiveThenForwardMissionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
