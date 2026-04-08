#!/usr/bin/env python3
"""
pixel_to_meter_delta_node.py

Purpose
-------
Convert the *cumulative* bottom-camera pose in pixels into:
  1) *incremental (delta)* motion in meters, and
  2) *cumulative (integrated)* motion in meters.

Why delta?
- /orca_auv/bottom_camera/total_transform_world is a cumulative pose [tx_px, ty_px, yaw, scale].
- If altitude (h) changes over time, converting the cumulative tx_px using the *current* h would be wrong.
- Therefore we compute per-update deltas:
    dtx_px = tx_now - tx_prev
    dty_px = ty_now - ty_prev
  then convert using the current h:
    dx_m = (dtx_px / fx) * h
    dy_m = (dty_px / fy) * h
  and integrate dx_m/dy_m to produce a meter-domain cumulative pose.

Inputs
------
Subscribe:
- bottom_camera/total_transform_world   (std_msgs/Float64MultiArray)
  Format: [tx_px, ty_px, yaw, scale]
- pressure_sensor_depth_m              (std_msgs/Float32)
  Depth convention: positive downward, meters from water surface.

Height to bottom (altitude)
---------------------------
Assume a fixed pool water depth D = water_depth_m (default 1.0 m).
Compute height-to-bottom:
    h = D - depth
Clamp:
    h = max(h, min_h_m)

Outputs
-------
Publish:
- bottom_camera/delta_transform_m      (std_msgs/Float64MultiArray)
  Format: [dx_m, dy_m, dyaw_rad, scale]
- bottom_camera/total_transform_world_m (std_msgs/Float64MultiArray)
  Format: [x_m, y_m, yaw_rad, scale]
  where x_m/y_m are the integrated meter-domain displacement since startup (first received pose).

Notes
-----
- This node uses namespace='orca_auv', so relative topic names are prefixed with /orca_auv/.
- dyaw is computed as yaw_now - yaw_prev. If wrap_dyaw is enabled, dyaw is wrapped to [-pi, pi].
- The meter-domain cumulative pose integrates dx_m/dy_m; yaw is taken from the incoming cumulative yaw.
"""

from typing import Optional
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


class PixelToMeterDeltaNode(Node):
    def __init__(self):
        super().__init__('bottom_camera_pixel_to_meter_delta_node', namespace='orca_auv')

        # ---- Parameters (topics) ----
        self.declare_parameter('in_total_topic', 'bottom_camera/total_transform_world')
        self.declare_parameter('depth_topic', 'pressure_sensor_depth_m')

        self.declare_parameter('out_delta_topic', 'bottom_camera/delta_transform_m')
        self.declare_parameter('out_total_m_topic', 'bottom_camera/total_transform_world_m')

        # ---- Parameters (geometry) ----
        self.declare_parameter('water_depth_m', 2.2)
        self.declare_parameter('fx', 554.38)
        self.declare_parameter('fy', 554.38)

        # ---- Parameters (safety / quality) ----
        self.declare_parameter('min_h_m', 0.05)
        self.declare_parameter('scale_gate_enable', True)
        self.declare_parameter('scale_gate_threshold', 0.05)

        # ---- Parameters (dyaw behavior) ----
        self.declare_parameter('wrap_dyaw', False)  # wrap to [-pi, pi]

        self._in_total_topic = self.get_parameter('in_total_topic').value
        self._depth_topic = self.get_parameter('depth_topic').value

        self._out_delta_topic = self.get_parameter('out_delta_topic').value
        self._out_total_m_topic = self.get_parameter('out_total_m_topic').value

        self._water_depth_m = float(self.get_parameter('water_depth_m').value)
        self._fx = float(self.get_parameter('fx').value)
        self._fy = float(self.get_parameter('fy').value)

        self._min_h_m = float(self.get_parameter('min_h_m').value)
        self._scale_gate_enable = bool(self.get_parameter('scale_gate_enable').value)
        self._scale_gate_threshold = float(self.get_parameter('scale_gate_threshold').value)

        self._wrap_dyaw = bool(self.get_parameter('wrap_dyaw').value)

        # ---- State ----
        self._depth_m: Optional[float] = None

        # Previous cumulative pose (pixels / radians)
        self._prev_tx_px: Optional[float] = None
        self._prev_ty_px: Optional[float] = None
        self._prev_yaw: Optional[float] = None

        # Integrated meter-domain totals
        self._total_x_m: float = 0.0
        self._total_y_m: float = 0.0

        # ---- Pub/Sub ----
        self._pub_delta = self.create_publisher(Float64MultiArray, self._out_delta_topic, 10)
        self._pub_total_m = self.create_publisher(Float64MultiArray, self._out_total_m_topic, 10)

        self.create_subscription(Float32, self._depth_topic, self._on_depth, 10)
        self.create_subscription(Float64MultiArray, self._in_total_topic, self._on_total_px, 10)

        self.get_logger().info("PixelToMeterDeltaNode started (namespace='orca_auv').")
        self.get_logger().info(f"  Subscribe total(px): {self._in_total_topic}")
        self.get_logger().info(f"  Subscribe depth(m):  {self._depth_topic} (positive downward)")
        self.get_logger().info(f"  Publish delta(m):    {self._out_delta_topic}")
        self.get_logger().info(f"  Publish total(m):    {self._out_total_m_topic}")
        self.get_logger().info(f"  water_depth_m={self._water_depth_m:.3f}, fx={self._fx:.2f}, fy={self._fy:.2f}")

    def _on_depth(self, msg: Float32) -> None:
        """Store the latest depth measurement (positive downward)."""
        self._depth_m = float(msg.data)

    def _compute_h(self) -> Optional[float]:
        """Compute height-to-bottom h = water_depth_m - depth_m, clamped to min_h_m."""
        if self._depth_m is None:
            return None
        h = self._water_depth_m - self._depth_m
        if h < self._min_h_m:
            h = self._min_h_m
        return h

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _publish_delta(self, dx_m: float, dy_m: float, dyaw: float, scale: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(dx_m), float(dy_m), float(dyaw), float(scale)]
        self._pub_delta.publish(msg)

    def _publish_total_m(self, yaw: float, scale: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(self._total_x_m), float(self._total_y_m), float(yaw), float(scale)]
        self._pub_total_m.publish(msg)

    def _on_total_px(self, msg: Float64MultiArray) -> None:
        """
        Convert cumulative pixel pose to incremental meter delta and integrate:
          dtx_px = tx_now - tx_prev
          dty_px = ty_now - ty_prev
          dx_m = (dtx_px / fx) * h
          dy_m = (dty_px / fy) * h
        Publish:
          - delta: [dx_m, dy_m, dyaw, scale]
          - total: [total_x_m, total_y_m, yaw, scale]
        """
        data = list(msg.data)
        if len(data) < 4:
            self.get_logger().warn(f"Expected 4 floats [tx,ty,yaw,scale], got {len(data)}")
            return

        tx_px, ty_px, yaw, scale = float(data[0]), float(data[1]), float(data[2]), float(data[3])

        # Optional quality gate: scale should stay near 1.0 for reliable estimation
        if self._scale_gate_enable and abs(scale - 1.0) > self._scale_gate_threshold:
            self.get_logger().warn(f"Scale out of range: {scale:.3f} (gate={self._scale_gate_threshold}), skip")
            return

        h = self._compute_h()
        if h is None:
            self.get_logger().warn("No depth received yet, skip")
            return

        # First message: initialize previous values; publish total (0,0) if you want visibility
        if self._prev_tx_px is None or self._prev_ty_px is None or self._prev_yaw is None:
            self._prev_tx_px = tx_px
            self._prev_ty_px = ty_px
            self._prev_yaw = yaw
            self._publish_total_m(yaw=yaw, scale=scale)
            return

        dtx_px = tx_px - self._prev_tx_px
        dty_px = ty_px - self._prev_ty_px
        dyaw = yaw - self._prev_yaw
        if self._wrap_dyaw:
            dyaw = self._wrap_to_pi(dyaw)

        dx_m = (dtx_px / self._fx) * h
        dy_m = (dty_px / self._fy) * h

        # Publish delta
        self._publish_delta(dx_m=dx_m, dy_m=dy_m, dyaw=dyaw, scale=scale)

        # Integrate and publish total in meters
        self._total_x_m += dx_m
        self._total_y_m += dy_m
        self._publish_total_m(yaw=yaw, scale=scale)

        # Update previous cumulative pose
        self._prev_tx_px = tx_px
        self._prev_ty_px = ty_px
        self._prev_yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = PixelToMeterDeltaNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
