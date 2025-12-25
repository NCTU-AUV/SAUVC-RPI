#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool


class WaypointTargetPublisher(Node):
    """
    Subscribes to current position (Float64MultiArray: [x, y]).
    Publishes the current target point (Float64MultiArray: [tx, ty]) from a hard-coded list.
    When the AUV reaches the current target, it advances to the next one.
    After all waypoints are reached, it publishes done=True (Bool).
    """

    def __init__(self):
        super().__init__("waypoint_target_publisher")

        # --- Parameters (keep topics/tolerance configurable) ---
        self.declare_parameter("current_topic", "/orca_auv/bottom_camera/total_transform_px")
        self.declare_parameter("target_topic", "/orca_auv/target_point_px")
        self.declare_parameter("done_topic", "/orca_auv/target_done")

        # Arrival tolerance in X/Y
        self.declare_parameter("tol_x", 5.0)
        self.declare_parameter("tol_y", 5.0)

        # Debounce: require N consecutive hits within tolerance to consider "reached"
        self.declare_parameter("stable_count", 5)

        # Publish the first target immediately on startup
        self.declare_parameter("publish_first_immediately", True)

        self.current_topic = self.get_parameter("current_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.done_topic = self.get_parameter("done_topic").value

        self.tol_x = float(self.get_parameter("tol_x").value)
        self.tol_y = float(self.get_parameter("tol_y").value)
        self.stable_count_need = int(self.get_parameter("stable_count").value)
        self.publish_first_immediately = bool(self.get_parameter("publish_first_immediately").value)

        # =========================================================
        # Hard-coded waypoint list (x, y) in order
        # Modify this list to change the route.
        # =========================================================
        self.targets: List[Tuple[float, float]] = [
            (0.0,   0.0),
            (120.0, 0.0),
            (120.0, 80.0),
            (0.0,   80.0),
        ]
        # =========================================================

        # --- Internal state ---
        self.idx = 0                 # Current waypoint index
        self._stable_hit = 0         # Consecutive in-tolerance counter (debounce)
        self.current_x = 0.0
        self.current_y = 0.0

        # --- Publishers/Subscribers ---
        self.pub_target = self.create_publisher(Float64MultiArray, self.target_topic, 10)
        self.pub_done = self.create_publisher(Bool, self.done_topic, 10)

        self.sub_current = self.create_subscription(
            Float64MultiArray,
            self.current_topic,
            self._on_current,
            10
        )

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
        msg = Float64MultiArray()
        msg.data = [float(tx), float(ty)]
        self.pub_target.publish(msg)
        if log:
            self.get_logger().info(f"[target] idx={self.idx} -> ({tx:.3f}, {ty:.3f})")

    def _advance_target(self):
        """Advance to the next waypoint, or mark done if finished."""
        self.idx += 1
        self._stable_hit = 0

        if self.idx >= len(self.targets):
            self.get_logger().info("All targets reached. done=True")
            self._publish_done(True)
            return

        self.get_logger().info(f"Advance to next target: idx={self.idx}")
        self._publish_current_target(log=True)

    def _on_current(self, msg: Float64MultiArray):
        """
        Callback on current position updates.
        Checks whether the AUV is within tolerance of the current target.
        Uses debounce (stable_count) to avoid switching due to jitter.
        """
        data = msg.data
        if data is None or len(data) < 2:
            return

        cx = float(data[0])
        cy = float(data[1])
        if not (math.isfinite(cx) and math.isfinite(cy)):
            return

        self.current_x = cx
        self.current_y = cy

        # Already finished
        if self.idx >= len(self.targets):
            return

        tx, ty = self.targets[self.idx]
        dx = tx - self.current_x
        dy = ty - self.current_y

        in_tol = (abs(dx) < self.tol_x) and (abs(dy) < self.tol_y)

        if in_tol:
            self._stable_hit += 1
            if self._stable_hit >= self.stable_count_need:
                self.get_logger().info(
                    f"Reached idx={self.idx} current=({cx:.3f},{cy:.3f}) "
                    f"target=({tx:.3f},{ty:.3f})"
                )
                self._advance_target()
        else:
            # Reset debounce counter as soon as we leave the tolerance region
            if self._stable_hit != 0:
                self._stable_hit = 0


def main():
    rclpy.init()
    node = WaypointTargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
