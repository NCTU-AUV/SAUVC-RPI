#!/usr/bin/env python3
"""
delta_to_velocity_node.py

Subscribe:
- /orca_auv/bottom_camera/delta_transform_m (std_msgs/Float64MultiArray)
  Format: [dx_m, dy_m, dyaw_rad, scale]

Publish:
- /orca_auv/bottom_camera/velocity_mps (std_msgs/Float64MultiArray)
  Format: [vx_mps, vy_mps, wz_radps, scale]
  where:
    vx_mps = dx_m / dt
    vy_mps = dy_m / dt
    wz_radps = dyaw_rad / dt

Notes:
- dt is computed from local receipt time (ROS clock) because input has no header timestamp.
- Optional smoothing (EMA) and gating are provided to reduce noise.
"""

from typing import Optional
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DeltaToVelocityNode(Node):
    def __init__(self):
        super().__init__('bottom_camera_delta_to_velocity_node', namespace='orca_auv')

        # ---- Parameters (topics) ----
        self.declare_parameter('in_delta_topic', 'bottom_camera/delta_transform_m')
        self.declare_parameter('out_vel_topic', 'bottom_camera/velocity_mps')

        # ---- Parameters (timing / safety) ----
        self.declare_parameter('min_dt_sec', 1e-3)     # ignore unrealistically small dt
        self.declare_parameter('max_dt_sec', 1.0)      # ignore too large dt (pause / hiccup)

        # ---- Parameters (optional smoothing) ----
        self.declare_parameter('ema_enable', True)     # exponential moving average
        self.declare_parameter('ema_alpha', 0.35)      # 0..1, higher = less smoothing

        # ---- Parameters (optional gates) ----
        self.declare_parameter('speed_gate_enable', True)
        self.declare_parameter('max_speed_mps', 2.0)   # clamp/skip if exceeded
        self.declare_parameter('max_wz_radps', 3.0)    # clamp/skip if exceeded

        # ---- Load params ----
        self._in_delta_topic = str(self.get_parameter('in_delta_topic').value)
        self._out_vel_topic = str(self.get_parameter('out_vel_topic').value)

        self._min_dt = float(self.get_parameter('min_dt_sec').value)
        self._max_dt = float(self.get_parameter('max_dt_sec').value)

        self._ema_enable = bool(self.get_parameter('ema_enable').value)
        self._ema_alpha = float(self.get_parameter('ema_alpha').value)

        self._speed_gate_enable = bool(self.get_parameter('speed_gate_enable').value)
        self._max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self._max_wz_radps = float(self.get_parameter('max_wz_radps').value)

        # ---- State ----
        self._prev_t: Optional[float] = None  # seconds
        self._ema_vx: Optional[float] = None
        self._ema_vy: Optional[float] = None
        self._ema_wz: Optional[float] = None

        # ---- Pub/Sub ----
        self._pub_vel = self.create_publisher(Float64MultiArray, self._out_vel_topic, 10)
        self.create_subscription(Float64MultiArray, self._in_delta_topic, self._on_delta, 10)

        self.get_logger().info("DeltaToVelocityNode started (namespace='orca_auv').")
        self.get_logger().info(f"  Subscribe delta: {self._in_delta_topic}  ([dx,dy,dyaw,scale])")
        self.get_logger().info(f"  Publish  vel:   {self._out_vel_topic}  ([vx,vy,wz,scale])")

    def _now_sec(self) -> float:
        # ROS clock time in seconds (works with /clock in simulation)
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _ema(self, prev: Optional[float], x: float) -> float:
        if prev is None:
            return x
        a = self._ema_alpha
        return a * x + (1.0 - a) * prev

    def _publish(self, vx: float, vy: float, wz: float, scale: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(vx), float(vy), float(wz), float(scale)]
        self._pub_vel.publish(msg)

    def _on_delta(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 4:
            self.get_logger().warn(f"Expected 4 floats [dx,dy,dyaw,scale], got {len(data)}")
            return

        dx_m = float(data[0])
        dy_m = float(data[1])
        dyaw = float(data[2])
        scale = float(data[3])

        t = self._now_sec()
        if self._prev_t is None:
            self._prev_t = t
            return

        dt = t - self._prev_t
        self._prev_t = t

        if not math.isfinite(dt) or dt < self._min_dt or dt > self._max_dt:
            # ignore weird timing (startup, pause, hiccup)
            return

        vx = dx_m / dt
        vy = dy_m / dt
        wz = dyaw / dt

        # optional gates
        if self._speed_gate_enable:
            speed = math.hypot(vx, vy)
            if speed > self._max_speed_mps or abs(wz) > self._max_wz_radps:
                # 這裡我選擇「skip」避免瞬間噪聲炸掉控制器
                # 你也可以改成 clamp
                return

        # optional smoothing
        if self._ema_enable:
            self._ema_vx = self._ema(self._ema_vx, vx)
            self._ema_vy = self._ema(self._ema_vy, vy)
            self._ema_wz = self._ema(self._ema_wz, wz)
            vx, vy, wz = self._ema_vx, self._ema_vy, self._ema_wz

        self._publish(vx=vx, vy=vy, wz=wz, scale=scale)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaToVelocityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
