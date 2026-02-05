#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Wrench


class VelocityPIDControllerNode(Node):
    """
    PID speed controller that outputs a Wrench (force.y) for forward speed control.

    Derivative is computed on MEASUREMENT (not on error) to avoid derivative kick when setpoint changes.
    D term is low-pass filtered (1st order) to reduce noise amplification.

    Subscribes:
      - measured_topic (Float64MultiArray): default "/orca_auv/bottom_camera/velocity_mps"
          expected format: [vx, vy, wz] (customize via measured_index)
      - target_topic (Float64): optional live target speed, default "/orca_auv/target_speed_mps"
        (If you don't publish target_topic, it will use parameter target_speed_mps.)

    Publishes:
      - output_topic (Wrench): default "/velocity_ctr_wrench"
        Only force.y is used by default (forward).
    """

    def __init__(self):
        super().__init__("velocity_pid_controller_node")

        # ---- Topics ----
        self.declare_parameter("measured_topic", "/orca_auv/bottom_camera/velocity_mps")
        self.declare_parameter("target_topic", "/orca_auv/target_speed_mps")  # optional
        self.declare_parameter("output_topic", "/velocity_ctr_wrench")

        # ---- Measured velocity format ----
        self.declare_parameter("measured_index", 1)  # default: vy (forward if your convention is force.y forward)

        # ---- PID gains ----
        self.declare_parameter("kp", 8.0)
        self.declare_parameter("ki", 2.0)
        self.declare_parameter("kd", 1.0)

        # ---- D low-pass filter ----
        # cutoff frequency in Hz (bigger => less filtering)
        self.declare_parameter("d_cutoff_hz", 5.0)

        # ---- Limits / safety ----
        self.declare_parameter("max_force_N", 25.0)
        self.declare_parameter("min_force_N", -25.0)
        self.declare_parameter("i_limit", 10.0)      # integrator clamp
        self.declare_parameter("deadband_mps", 0.01)  # small error ignored

        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("measurement_timeout_s", 0.5)

        # Optional parameter-based target (if you don't publish target_topic)
        self.declare_parameter("target_speed_mps", 0.0)

        # ---- Load params ----
        self.measured_topic = self.get_parameter("measured_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.measured_index = int(self.get_parameter("measured_index").value)

        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)

        self.d_cutoff_hz = float(self.get_parameter("d_cutoff_hz").value)

        self.max_force = float(self.get_parameter("max_force_N").value)
        self.min_force = float(self.get_parameter("min_force_N").value)
        self.i_limit = float(self.get_parameter("i_limit").value)
        self.deadband = float(self.get_parameter("deadband_mps").value)

        self.control_rate = float(self.get_parameter("control_rate_hz").value)
        self.timeout_s = float(self.get_parameter("measurement_timeout_s").value)

        self._target_speed = float(self.get_parameter("target_speed_mps").value)

        # ---- State ----
        self._measured_speed: Optional[float] = None
        self._last_meas_time = None  # rclpy.time.Time

        self._integrator = 0.0

        # For derivative on measurement
        self._prev_measured_speed: Optional[float] = None
        self._d_filt = 0.0  # filtered derivative (m/s^2)

        self._last_control_time = self.get_clock().now()

        # ---- Pub/Sub ----
        self.pub = self.create_publisher(Wrench, self.output_topic, 10)

        self.sub_meas = self.create_subscription(
            Float64MultiArray,
            self.measured_topic,
            self._on_measured,
            10,
        )

        self.sub_target = self.create_subscription(
            Float64,
            self.target_topic,
            self._on_target,
            10,
        )

        period = 1.0 / max(self.control_rate, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info("VelocityPIDControllerNode started")
        self.get_logger().info(f"  measured_topic: {self.measured_topic} (index={self.measured_index})")
        self.get_logger().info(f"  target_topic:   {self.target_topic} (optional)")
        self.get_logger().info(f"  output_topic:   {self.output_topic}")
        self.get_logger().info(f"  kp={self.kp:.3f}, ki={self.ki:.3f}, kd={self.kd:.3f}, d_cutoff_hz={self.d_cutoff_hz:.2f}")
        self.get_logger().info(f"  force clamp: [{self.min_force:.2f}, {self.max_force:.2f}] N, i_limit={self.i_limit:.2f}")

    def _on_measured(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) <= self.measured_index:
            self.get_logger().warn(f"measured msg too short: len={len(data)} need index={self.measured_index}")
            return

        v = float(data[self.measured_index])
        if not math.isfinite(v):
            return

        self._measured_speed = v
        self._last_meas_time = self.get_clock().now()

    def _on_target(self, msg: Float64):
        v = float(msg.data)
        if math.isfinite(v):
            self._target_speed = v

    def _publish_force_y(self, force_y: float):
        w = Wrench()
        w.force.x = 0.0
        w.force.y = float(force_y)  # forward (your repo convention)
        w.force.z = 0.0
        w.torque.x = 0.0
        w.torque.y = 0.0
        w.torque.z = 0.0
        self.pub.publish(w)

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_control_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / max(self.control_rate, 1e-3)
        self._last_control_time = now

        # No measurement yet or timeout => output 0 and decay integrator
        if self._measured_speed is None or self._last_meas_time is None:
            self._integrator *= 0.9
            self._prev_measured_speed = None
            self._d_filt = 0.0
            self._publish_force_y(0.0)
            return

        age = (now - self._last_meas_time).nanoseconds * 1e-9
        if age > self.timeout_s:
            self._integrator *= 0.9
            self._prev_measured_speed = None
            self._d_filt = 0.0
            self._publish_force_y(0.0)
            return

        measured = self._measured_speed
        target = self._target_speed
        err = target - measured

        if abs(err) < self.deadband:
            err = 0.0

        # ---- Derivative on measurement (avoid D kick) ----
        # d(measured)/dt
        d_meas = 0.0
        if self._prev_measured_speed is not None and dt > 1e-6:
            d_meas = (measured - self._prev_measured_speed) / dt
        self._prev_measured_speed = measured

        # Low-pass filter derivative (1st order)
        # y += alpha*(x - y), alpha = dt / (tau + dt), tau = 1/(2*pi*fc)
        fc = max(self.d_cutoff_hz, 1e-6)
        tau = 1.0 / (2.0 * math.pi * fc)
        alpha = dt / (tau + dt)
        self._d_filt = self._d_filt + alpha * (d_meas - self._d_filt)

        # D term uses negative derivative of measurement
        d_term = -self.kd * self._d_filt

        # ---- PID (with simple anti-windup) ----
        # compute unsaturated output first
        u_unsat = (self.kp * err) + (self.ki * self._integrator) + d_term
        u_sat = self._clamp(u_unsat, self.min_force, self.max_force)

        # Anti-windup: if saturated and error would push further into saturation, don't integrate
        pushing_further = (u_unsat != u_sat) and ((u_unsat - u_sat) * err > 0.0)
        if not pushing_further:
            self._integrator += err * dt
            self._integrator = self._clamp(self._integrator, -self.i_limit, self.i_limit)

        # final output
        u = (self.kp * err) + (self.ki * self._integrator) + d_term
        u = self._clamp(u, self.min_force, self.max_force)

        self._publish_force_y(u)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPIDControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
