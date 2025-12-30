#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Direction / Rotation PID Controller (ROS2 rclpy)

用途：
- 訂閱 total_transform_world（Float64MultiArray）取得目前位置與 rotation(yaw)
- 計算「目標方向 direction」(rad)
  - target_mode=point：訂閱 target_point_world（Float64MultiArray: [x, y]），direction = atan2(ty-y, tx-x)
  - target_mode=angle：訂閱 target_direction_rad（Float64），direction = 目標方向角
- PID 控制 yaw 對準 direction，輸出到自己的 wrench topic（預設 /direction_ctr_wrench）
- 最後由 wrench_sum_node 加總後送到 /orca_auv/set_output_wrench_at_center_N_Nm

發布：
- geometry_msgs/Wrench
  - torque.z = tau (N·m)
  - 其餘 force/torque 設 0
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class DirectionYawPidController(Node):
    """
    控制目標：讓目前 rotation(yaw) 對準「direction」（方向角）

    Sub:
      - total_transform_topic (Float64MultiArray): 取 x, y, rotation(yaw)
      - target_point_topic (Float64MultiArray): 目標點 [x, y] (target_mode='point')
      - target_direction_topic (Float64): 目標方向角 (target_mode='angle')

    Pub:
      - output_topic (geometry_msgs/Wrench): torque.z
    """

    def __init__(self):
        super().__init__("direction_yaw_pid_controller")

        # ------------------------
        # Topics / indices
        # ------------------------
        self.declare_parameter("total_transform_topic", "/orca_auv/total_transform_world")
        self.declare_parameter("x_index", 0)
        self.declare_parameter("y_index", 1)
        self.declare_parameter("rot_index", 2)  # rotation/yaw (rad)

        # target_mode: 'point' or 'angle'
        self.declare_parameter("target_mode", "point")

        # point mode topic + indices
        self.declare_parameter("target_point_topic", "/orca_auv/target_point_world")
        self.declare_parameter("target_point_x_index", 0)
        self.declare_parameter("target_point_y_index", 1)

        # angle mode topic
        self.declare_parameter("target_direction_topic", "/orca_auv/target_direction_rad")

        # IMPORTANT: publish to your own wrench topic for wrench_sum to subscribe
        self.declare_parameter("output_topic", "/direction_ctr_wrench")

        # ------------------------
        # PID parameters
        # ------------------------
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.2)

        # control loop
        self.declare_parameter("rate_hz", 100.0)

        # ------------------------
        # Safety / behavior
        # ------------------------
        self.declare_parameter("max_torque_z", 2.0)       # torque.z saturation (N·m)
        self.declare_parameter("integral_limit", 1.0)     # |I_state| limit
        self.declare_parameter("deadband_rad", 0.01)      # small error -> output 0
        self.declare_parameter("reset_i_on_deadband", True)
        self.declare_parameter("d_filter_alpha", 0.7)     # derivative LPF alpha [0,1)

        # point mode: distance threshold to avoid jitter when very close
        self.declare_parameter("min_target_distance", 0.05)  # meter

        # If no target available, output zero
        self.declare_parameter("output_zero_when_no_target", True)

        # ------------------------
        # State
        # ------------------------
        self._have_pose = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._have_target_point = False
        self._tx = 0.0
        self._ty = 0.0

        self._have_target_dir = False
        self._target_dir = 0.0

        self._i_state = 0.0
        self._prev_error = 0.0
        self._d_filt = 0.0
        self._prev_time = self.get_clock().now()

        # ------------------------
        # Publisher / Subscribers
        # ------------------------
        self._pub = self.create_publisher(
            Wrench,
            self.get_parameter("output_topic").value,
            10
        )

        self._sub_pose = self.create_subscription(
            Float64MultiArray,
            self.get_parameter("total_transform_topic").value,
            self._on_pose,
            10
        )

        # subscribe both; choose by target_mode at runtime
        self._sub_tpoint = self.create_subscription(
            Float64MultiArray,
            self.get_parameter("target_point_topic").value,
            self._on_target_point,
            10
        )

        self._sub_tdir = self.create_subscription(
            Float64,
            self.get_parameter("target_direction_topic").value,
            self._on_target_dir,
            10
        )

        rate_hz = float(self.get_parameter("rate_hz").value)
        period = 1.0 / max(rate_hz, 1e-6)
        self._timer = self.create_timer(period, self._on_timer)

        # allow dynamic param updates (basic validation)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            "direction_yaw_pid_controller started\n"
            f"  total_transform_topic: {self.get_parameter('total_transform_topic').value}\n"
            f"  target_mode: {self.get_parameter('target_mode').value}\n"
            f"  output_topic: {self.get_parameter('output_topic').value}\n"
            f"  kp/ki/kd: {self.get_parameter('kp').value}/"
            f"{self.get_parameter('ki').value}/"
            f"{self.get_parameter('kd').value}\n"
            f"  rate_hz: {rate_hz}"
        )

    # ------------------------
    # Parameter validation
    # ------------------------
    def _on_params(self, params):
        for p in params:
            if p.name == "rate_hz":
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False, reason="rate_hz must be > 0")

            if p.name == "d_filter_alpha":
                a = float(p.value)
                if not (0.0 <= a < 1.0):
                    return SetParametersResult(successful=False, reason="d_filter_alpha must be in [0, 1)")

            if p.name == "target_mode":
                v = str(p.value)
                if v not in ("point", "angle"):
                    return SetParametersResult(successful=False, reason="target_mode must be 'point' or 'angle'")

        return SetParametersResult(successful=True)

    # ------------------------
    # Callbacks
    # ------------------------
    def _on_pose(self, msg: Float64MultiArray):
        data = msg.data
        if data is None:
            return

        xi = int(self.get_parameter("x_index").value)
        yi = int(self.get_parameter("y_index").value)
        ri = int(self.get_parameter("rot_index").value)

        if min(xi, yi, ri) < 0:
            return
        if len(data) <= max(xi, yi, ri):
            return

        x = float(data[xi])
        y = float(data[yi])
        yaw = float(data[ri])

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return

        self._x, self._y, self._yaw = x, y, yaw
        self._have_pose = True

    def _on_target_point(self, msg: Float64MultiArray):
        data = msg.data
        if data is None:
            return

        xi = int(self.get_parameter("target_point_x_index").value)
        yi = int(self.get_parameter("target_point_y_index").value)

        if min(xi, yi) < 0:
            return
        if len(data) <= max(xi, yi):
            return

        tx = float(data[xi])
        ty = float(data[yi])

        if not (math.isfinite(tx) and math.isfinite(ty)):
            return

        self._tx, self._ty = tx, ty
        self._have_target_point = True

    def _on_target_dir(self, msg: Float64):
        v = float(msg.data)
        if not math.isfinite(v):
            return
        self._target_dir = v
        self._have_target_dir = True

    # ------------------------
    # Compute target direction
    # ------------------------
    def _compute_target_direction(self) -> Optional[float]:
        mode = str(self.get_parameter("target_mode").value)

        if mode == "angle":
            if not self._have_target_dir:
                return None
            return self._target_dir

        # mode == "point"
        if not (self._have_pose and self._have_target_point):
            return None

        dx = self._tx - self._x
        dy = self._ty - self._y
        dist = math.hypot(dx, dy)

        if dist < float(self.get_parameter("min_target_distance").value):
            return None

        return math.atan2(dy, dx)

    # ------------------------
    # Control loop
    # ------------------------
    def _on_timer(self):
        if not self._have_pose:
            return

        target_dir = self._compute_target_direction()
        if target_dir is None:
            if bool(self.get_parameter("output_zero_when_no_target").value):
                self._publish_torque_z(0.0)
            return

        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # params
        kp = float(self.get_parameter("kp").value)
        ki = float(self.get_parameter("ki").value)
        kd = float(self.get_parameter("kd").value)

        max_tz = float(self.get_parameter("max_torque_z").value)
        i_lim = float(self.get_parameter("integral_limit").value)
        deadband = float(self.get_parameter("deadband_rad").value)
        reset_i_on_deadband = bool(self.get_parameter("reset_i_on_deadband").value)
        d_alpha = float(self.get_parameter("d_filter_alpha").value)

        # error: direction - yaw
        error = wrap_to_pi(target_dir - self._yaw)

        # deadband
        if abs(error) < deadband:
            if reset_i_on_deadband:
                self._i_state = 0.0
            self._publish_torque_z(0.0)
            self._prev_error = error
            self._prev_time = now
            return

        # derivative with LPF
        derr = (error - self._prev_error) / dt
        self._d_filt = d_alpha * self._d_filt + (1.0 - d_alpha) * derr

        p = kp * error
        d = kd * self._d_filt

        # anti-windup: conditional integration
        i_candidate = clamp(self._i_state + error * dt, -i_lim, i_lim)
        u_unsat = p + ki * i_candidate + d

        saturating = abs(u_unsat) > max_tz
        pushing_further = (u_unsat > 0.0 and error > 0.0) or (u_unsat < 0.0 and error < 0.0)
        if not (saturating and pushing_further):
            self._i_state = i_candidate

        u = clamp(p + ki * self._i_state + d, -max_tz, max_tz)
        self._publish_torque_z(u)

        self._prev_error = error
        self._prev_time = now

    # ------------------------
    # Publish
    # ------------------------
    def _publish_torque_z(self, tz: float):
        msg = Wrench()
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = 0.0
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = float(tz)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = DirectionYawPidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
