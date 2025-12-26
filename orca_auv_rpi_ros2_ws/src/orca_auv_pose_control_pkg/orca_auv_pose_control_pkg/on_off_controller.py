#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


def _sign(v: float) -> float:
    """Return sign of v: +1, -1, or 0."""
    if v > 0.0:
        return 1.0
    elif v < 0.0:
        return -1.0
    else:
        return 0.0


class OnOffController(Node):
    """
    Subscribes:
      - current_topic: Float64MultiArray [cx, cy, yaw] (yaw optional; used if present)
      - target_topic : Float64MultiArray [tx, ty]

    Publishes:
      - output_topic: geometry_msgs/Wrench
        force.x: positive means move right, negative means move left
        force.y: positive means move forward, negative means move backward

    Logic:
      - If error exceeds tolerance, output fixed thrust (on/off).
      - If single_axis_only=True, move one axis at a time (Y first, then X).
      - If single_axis_only=False, allow X and Y thrust simultaneously.
    """

    def __init__(self):
        super().__init__("on_off_controller")

        # --- Parameters ---
        self.declare_parameter("current_topic", "/orca_auv/bottom_camera/total_transform_px")
        self.declare_parameter("target_topic", "/orca_auv/target_point_px")
        self.declare_parameter("output_topic", "/orca_auv/set_output_wrench_at_center_N_Nm")
        # Index of yaw (rad) within current_topic; set negative to ignore orientation
        self.declare_parameter("yaw_index", 2)

        # Optional initial target (will be overwritten when target_topic is received)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)

        # Tolerances
        self.declare_parameter("tol_x", 5.0)
        self.declare_parameter("tol_y", 5.0)

        # Fixed on/off thrust magnitude
        self.declare_parameter("thrust", 10.0)

        # Control mode
        self.declare_parameter("single_axis_only", True)

        self.current_topic = self.get_parameter("current_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.yaw_index = int(self.get_parameter("yaw_index").value)

        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)

        self.tol_x = float(self.get_parameter("tol_x").value)
        self.tol_y = float(self.get_parameter("tol_y").value)

        self.thrust = float(self.get_parameter("thrust").value)
        self.single_axis_only = bool(self.get_parameter("single_axis_only").value)

        # --- State ---
        self.current_x = 0.0
        self.current_y = 0.0
        self._have_current = False
        self._have_orientation = False
        self._yaw = 0.0

        # --- Pub/Sub ---
        self.pub_wrench = self.create_publisher(Wrench, self.output_topic, 10)

        self.sub_current = self.create_subscription(
            Float64MultiArray,
            self.current_topic,
            self._on_current,
            10,
        )

        self.sub_target = self.create_subscription(
            Float64MultiArray,
            self.target_topic,
            self._on_target,
            10,
        )

        # Allow dynamic parameter updates (optional)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            f"current: {self.current_topic}\n"
            f"target : {self.target_topic}\n"
            f"output : {self.output_topic}\n"
            f"init target=({self.target_x},{self.target_y}) tol=({self.tol_x},{self.tol_y}) "
            f"thrust={self.thrust} single_axis_only={self.single_axis_only} yaw_index={self.yaw_index}"
        )

    def _on_params(self, params):
        """Handle dynamic parameter updates."""
        for p in params:
            if p.name == "target_x":
                self.target_x = float(p.value)
            elif p.name == "target_y":
                self.target_y = float(p.value)
            elif p.name == "tol_x":
                self.tol_x = float(p.value)
            elif p.name == "tol_y":
                self.tol_y = float(p.value)
            elif p.name == "thrust":
                self.thrust = float(p.value)
            elif p.name == "single_axis_only":
                self.single_axis_only = bool(p.value)
        return SetParametersResult(successful=True)

    def _on_target(self, msg: Float64MultiArray):
        """Update target point from target_topic."""
        data = msg.data
        if data is None or len(data) < 2:
            return

        tx = float(data[0])
        ty = float(data[1])
        if not (math.isfinite(tx) and math.isfinite(ty)):
            return

        self.target_x = tx
        self.target_y = ty
        self.get_logger().info(f"[target update] ({tx:.3f}, {ty:.3f})")

        # Recompute immediately when target changes (if we already have current)
        if self._have_current:
            self._compute_and_publish()

    def _on_current(self, msg: Float64MultiArray):
        """Update current point from current_topic and publish on/off thrust."""
        data = msg.data
        if data is None or len(data) < 2:
            return

        cx = float(data[0])
        cy = float(data[1])
        if not (math.isfinite(cx) and math.isfinite(cy)):
            return

        self.current_x = cx
        self.current_y = cy
        self._have_current = True

        if self.yaw_index >= 0 and len(data) > self.yaw_index:
            yaw = float(data[self.yaw_index])
            if math.isfinite(yaw):
                self._yaw = yaw
                self._have_orientation = True

        self._compute_and_publish()

    def _compute_and_publish(self):
        """Compute fixed on/off thrust based on target-current error and publish Wrench."""
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        need_x = abs(dx) >= self.tol_x
        need_y = abs(dy) >= self.tol_y

        fx_world = 0.0
        fy_world = 0.0

        if self.single_axis_only:
            # Move Y axis first; only move X when Y is within tolerance
            if need_y:
                fy_world = self.thrust * _sign(dy)
            elif need_x:
                fx_world = self.thrust * _sign(dx)
        else:
            # Move both axes if needed
            if need_x:
                fx_world = self.thrust * _sign(dx)
            if need_y:
                fy_world = self.thrust * _sign(dy)

        if self._have_orientation:
            # Rotate desired world-frame thrust into the vehicle/body frame using yaw
            cos_yaw = math.cos(-self._yaw)
            sin_yaw = math.sin(-self._yaw)
            fx = fx_world * cos_yaw - fy_world * sin_yaw
            fy = fx_world * sin_yaw + fy_world * cos_yaw
        else:
            fx = fx_world
            fy = fy_world

        w = Wrench()
        w.force.x = float(fx)
        w.force.y = float(fy)
        w.force.z = 0.0
        w.torque.x = 0.0
        w.torque.y = 0.0
        w.torque.z = 0.0
        self.pub_wrench.publish(w)


def main():
    rclpy.init()
    node = OnOffController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
