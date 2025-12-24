#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import SetParametersResult

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


class GoalPointThrustNode(Node):
    """
    訂閱 bottom_camera 的 total_transform_px → 取得 current_point(x,y)
    設定 target_point(x,y) → 計算 error = target - current
    用 on/off 固定推力輸出到 /orca_auv/set_output_wrench_at_center_N_Nm

    座標：
      - 前進：force.y > 0
      - 後退：force.y < 0
      - 向右側移：force.x > 0
      - 向左側移：force.x < 0
    """

    def __init__(self):
        super().__init__('goal_point_thrust_node')

        # --- Params ---
        self.declare_parameter('input_topic', '/orca_auv/bottom_camera/total_transform_px')
        self.declare_parameter('output_topic', '/orca_auv/set_output_wrench_at_center_N_Nm')

        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)

        self.declare_parameter('tol_x', 5.0)
        self.declare_parameter('tol_y', 5.0)

        self.declare_parameter('thrust', 2.0)

        # 允許把座標方向/判斷邏輯改成「同時給 Fx+Fy」也行，
        # 但目前先保留：一次只推一個方向（更單純）
        self.declare_parameter('single_axis_only', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.tol_x = float(self.get_parameter('tol_x').value)
        self.tol_y = float(self.get_parameter('tol_y').value)
        self.thrust = float(self.get_parameter('thrust').value)
        self.single_axis_only = bool(self.get_parameter('single_axis_only').value)

        # current 由 total_transform 直接給（不在本 node 內累積）
        self.current_x = 0.0
        self.current_y = 0.0

        self.add_on_set_parameters_callback(self._on_params)

        self.pub_wrench = self.create_publisher(Wrench, self.output_topic, 10)
        self.sub = self.create_subscription(
            Float64MultiArray,
            self.input_topic,
            self._on_total_transform,
            10
        )

        self.get_logger().info(
            f"listening: {self.input_topic}  | publishing wrench: {self.output_topic}"
        )

    def _on_params(self, params):
        for p in params:
            if p.name == 'target_x':
                self.target_x = float(p.value)
            elif p.name == 'target_y':
                self.target_y = float(p.value)
            elif p.name == 'tol_x':
                self.tol_x = float(p.value)
            elif p.name == 'tol_y':
                self.tol_y = float(p.value)
            elif p.name == 'thrust':
                self.thrust = float(p.value)
            elif p.name == 'single_axis_only':
                self.single_axis_only = bool(p.value)
        return SetParametersResult(successful=True)

    def _on_total_transform(self, msg: Float64MultiArray):
        data = msg.data

        cx = float(data[0])
        cy = float(data[1])

        if not (math.isfinite(cx) and math.isfinite(cy)):
            return

        self.current_x = cx
        self.current_y = cy

        # error = target - current
        ex = self.target_x - self.current_x
        ey = self.target_y - self.current_y

        fx = 0.0
        fy = 0.0

        # 到點：停
        if abs(ex) < self.tol_x and abs(ey) < self.tol_y:
            fx, fy = 0.0, 0.0
        else:
            if self.single_axis_only:
                # 一次只推一個方向：看哪個誤差比較大
                if abs(ey) >= abs(ex):
                    # 前/後：用 force.y（前進 y>0）
                    fy = self.thrust if ey > 0.0 else -self.thrust
                else:
                    # 左/右：用 force.x（右為正 x>0）
                    fx = self.thrust if ex > 0.0 else -self.thrust
            else:
                # 允許斜向：同時推 x 與 y（仍然是固定推力 on/off）
                if abs(ex) >= self.tol_x:
                    fx = self.thrust if ex > 0.0 else -self.thrust
                if abs(ey) >= self.tol_y:
                    fy = self.thrust if ey > 0.0 else -self.thrust

        w = Wrench()
        w.force.x = fx
        w.force.y = fy
        w.force.z = 0.0
        w.torque.x = 0.0
        w.torque.y = 0.0
        w.torque.z = 0.0
        self.pub_wrench.publish(w)


def main():
    rclpy.init()
    node = GoalPointThrustNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
