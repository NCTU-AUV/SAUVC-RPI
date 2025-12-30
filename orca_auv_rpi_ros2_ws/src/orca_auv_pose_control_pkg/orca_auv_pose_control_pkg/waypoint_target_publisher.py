#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool


class WaypointTargetPublisher(Node):
    """
    Publishes a feed-forward target path (Float64MultiArray: [tx, ty]) along a hard-coded list.
    Does not use feedback; it simply walks the path at a constant speed and raises done=True (Bool) at the end.
    """

    def __init__(self):
        super().__init__("waypoint_target_publisher")

        # --- Parameters (keep topics/tolerance configurable) ---
        self.declare_parameter("current_topic", "/orca_auv/bottom_camera/total_transform_world")
        self.declare_parameter("target_topic", "/orca_auv/target_point_px")
        self.declare_parameter("done_topic", "/orca_auv/target_done")

        # Desired speed for interpolated setpoints (units/sec, same units as waypoints)
        self.declare_parameter("setpoint_speed", 20.0)
        # Timer period for publishing interpolated setpoints
        self.declare_parameter("publish_period", 0.1)

        # Publish the first target immediately on startup
        self.declare_parameter("publish_first_immediately", True)

        self.current_topic = self.get_parameter("current_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.done_topic = self.get_parameter("done_topic").value

        self.setpoint_speed = float(self.get_parameter("setpoint_speed").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.publish_first_immediately = bool(self.get_parameter("publish_first_immediately").value)

        # =========================================================
        # Hard-coded waypoint list (x, y) in order
        # Modify this list to change the route.
        # =========================================================
        self.targets: List[Tuple[float, float]] = [
            (0.0,   0.0),
            (2000.0, 0.0),
            (2000.0, -1000.0),
            (0.0, -1000.0),
            (0.0,   0.0),
        ]
        # =========================================================

        # --- Internal state ---
        self.idx = 0                 # Current waypoint index
        self._next_idx = 1 if len(self.targets) > 1 else None
        self._current_point = self.targets[0] if self.targets else (0.0, 0.0)
        self._last_time = None       # Time of last timer tick
        self._done_published = False

        # --- Publishers/Subscribers ---
        self.pub_target = self.create_publisher(Float64MultiArray, self.target_topic, 10)
        self.pub_done = self.create_publisher(Bool, self.done_topic, 10)

        if self.publish_period > 0.0:
            self.timer = self.create_timer(self.publish_period, self._on_timer)
        else:
            self.get_logger().warn("publish_period <= 0; no timer created, targets will not be published.")

        # Startup behavior
        if not self.targets:
            self.get_logger().warn("targets list is empty. No target will be published.")
            self._publish_done(True)
        else:
            self._publish_done(False)
            if self.publish_first_immediately:
                self._publish_current_target(log=True)

        self.get_logger().info(
            f"current: {self.current_topic}\n"
            f"target : {self.target_topic}\n"
            f"done   : {self.done_topic}\n"
            f"targets({len(self.targets)}): {self.targets}"
        )

    def _publish_done(self, done: bool):
        """Publish mission done flag."""
        msg = Bool()
        msg.data = bool(done)
        self.pub_done.publish(msg)

    def _publish_current_target(self, log: bool = False):
        """Publish the current waypoint target (tx, ty)."""
        if self.idx >= len(self.targets):
            return
        tx, ty = self.targets[self.idx]
        self._publish_target_point(tx, ty, log=log)

    def _publish_target_point(self, tx: float, ty: float, log: bool = False):
        """Publish an arbitrary target point."""
        msg = Float64MultiArray()
        msg.data = [float(tx), float(ty)]
        self.pub_target.publish(msg)
        if log:
            self.get_logger().info(f"[target] idx={self.idx} -> ({tx:.3f}, {ty:.3f})")

    def _advance_target(self):
        """Advance to the next waypoint, or mark done if finished."""
        self.idx += 1
        self._next_idx = self.idx + 1 if self.idx + 1 < len(self.targets) else None
        if self.idx >= len(self.targets) - 1:
            if not self._done_published:
                self.get_logger().info("All targets published. done=True")
                self._publish_done(True)
                self._done_published = True
            return

        self.get_logger().info(f"Advance to next target: idx={self.idx}")
        self._publish_current_target(log=True)

    def _on_timer(self):
        """Timer callback: walk the path at constant speed, no feedback."""
        if not self.targets:
            return

        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if dt <= 0.0 or self.setpoint_speed <= 0.0:
            self._publish_target_point(*self._current_point)
            return

        remaining_step = self.setpoint_speed * dt

        while remaining_step > 0.0 and self._next_idx is not None:
            tx, ty = self.targets[self._next_idx]
            cx, cy = self._current_point
            dx = tx - cx
            dy = ty - cy
            distance = math.hypot(dx, dy)

            if distance < 1e-9:
                # Already at the next waypoint, advance.
                self._current_point = (tx, ty)
                self._advance_target()
                continue

            if distance <= remaining_step:
                # Reach the waypoint in this cycle.
                self._current_point = (tx, ty)
                remaining_step -= distance
                self._advance_target()
            else:
                # Move along the segment by the remaining step.
                scale = remaining_step / distance
                self._current_point = (cx + dx * scale, cy + dy * scale)
                remaining_step = 0.0

        # Publish current interpolated point (or last waypoint if done)
        self._publish_target_point(*self._current_point)


def main():
    rclpy.init()
    node = WaypointTargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
