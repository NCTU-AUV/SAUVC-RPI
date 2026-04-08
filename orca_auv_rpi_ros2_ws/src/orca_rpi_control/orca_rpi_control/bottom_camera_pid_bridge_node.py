import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


class BottomCameraPIDBridgeNode(Node):
    """
    Bridge between bottom camera targets/measurements and the generic PID controllers.

    Responsibilities:
      - Split Float64MultiArray target/current into per-axis Float64 topics for PID setpoints and feedback.
      - Collect per-axis PID outputs and rotate world-frame forces into the AUV body frame using yaw.
      - Publish the combined forces as a geometry_msgs/Wrench to drive the vehicle.
    """

    def __init__(self):
        super().__init__("bottom_camera_pid_bridge")

        # Input/output topics
        self.declare_parameter("current_topic", "/orca_auv/bottom_camera/total_transform_world")
        self.declare_parameter("target_topic", "/orca_auv/target_point_px")
        self.declare_parameter("output_topic", "/orca_auv/set_output_wrench_at_center_N_Nm")
        self.declare_parameter("yaw_index", 2)

        # Optional initial target
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)

        # PID wiring topics
        self.declare_parameter("x_reference_topic", "/orca_auv/bottom_camera/x_reference_input")
        self.declare_parameter("y_reference_topic", "/orca_auv/bottom_camera/y_reference_input")
        self.declare_parameter("x_feedback_topic", "/orca_auv/bottom_camera/x_output_feedback")
        self.declare_parameter("y_feedback_topic", "/orca_auv/bottom_camera/y_output_feedback")
        self.declare_parameter("x_manipulated_topic", "/orca_auv/bottom_camera/x_force_world_N")
        self.declare_parameter("y_manipulated_topic", "/orca_auv/bottom_camera/y_force_world_N")

        self.current_topic = self.get_parameter("current_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.yaw_index = int(self.get_parameter("yaw_index").value)

        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self._have_target = True

        self.x_reference_topic = self.get_parameter("x_reference_topic").value
        self.y_reference_topic = self.get_parameter("y_reference_topic").value
        self.x_feedback_topic = self.get_parameter("x_feedback_topic").value
        self.y_feedback_topic = self.get_parameter("y_feedback_topic").value
        self.x_manipulated_topic = self.get_parameter("x_manipulated_topic").value
        self.y_manipulated_topic = self.get_parameter("y_manipulated_topic").value

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self._have_current = False
        self._yaw = 0.0
        self._have_orientation = False

        self._x_force_world = 0.0
        self._y_force_world = 0.0
        self._have_x_force = False
        self._have_y_force = False

        # Pub/Sub to interface with PID controllers
        self.pub_x_reference = self.create_publisher(Float64, self.x_reference_topic, 10)
        self.pub_y_reference = self.create_publisher(Float64, self.y_reference_topic, 10)
        self.pub_x_feedback = self.create_publisher(Float64, self.x_feedback_topic, 10)
        self.pub_y_feedback = self.create_publisher(Float64, self.y_feedback_topic, 10)
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
        self.sub_x_manipulated = self.create_subscription(
            Float64,
            self.x_manipulated_topic,
            self._on_x_output,
            10,
        )
        self.sub_y_manipulated = self.create_subscription(
            Float64,
            self.y_manipulated_topic,
            self._on_y_output,
            10,
        )

        # Allow dynamic updates for the target setpoint
        self.add_on_set_parameters_callback(self._on_params)

        # Publish the initial target so the PID controllers start with a valid setpoint
        self._publish_reference_inputs()

        self.get_logger().info(
            f"current: {self.current_topic}\n"
            f"target : {self.target_topic}\n"
            f"output : {self.output_topic}\n"
            f"x_ref : {self.x_reference_topic} y_ref: {self.y_reference_topic}\n"
            f"x_fb  : {self.x_feedback_topic} y_fb : {self.y_feedback_topic}\n"
            f"x_out : {self.x_manipulated_topic} y_out: {self.y_manipulated_topic}"
        )

    def _on_params(self, params):
        """Handle dynamic parameter updates for target_x/target_y."""
        updated = False
        for p in params:
            if p.name == "target_x":
                self.target_x = float(p.value)
                updated = True
            elif p.name == "target_y":
                self.target_y = float(p.value)
                updated = True
        if updated:
            self._publish_reference_inputs()
        return SetParametersResult(successful=True)

    def _on_target(self, msg: Float64MultiArray):
        data = msg.data
        if data is None or len(data) < 2:
            return

        tx = float(data[0])
        ty = float(data[1])
        if not (math.isfinite(tx) and math.isfinite(ty)):
            return

        self.target_x = tx
        self.target_y = ty
        self._have_target = True
        self._publish_reference_inputs()

    def _on_current(self, msg: Float64MultiArray):
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
            yaw_val = float(data[self.yaw_index])
            if math.isfinite(yaw_val):
                self._yaw = yaw_val
                self._have_orientation = True

        self._publish_feedback_inputs()
        self._maybe_publish_wrench()

    def _on_x_output(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._x_force_world = float(msg.data)
        self._have_x_force = True
        self._maybe_publish_wrench()

    def _on_y_output(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self._y_force_world = float(msg.data)
        self._have_y_force = True
        self._maybe_publish_wrench()

    def _publish_reference_inputs(self):
        if not self._have_target:
            return
        x_msg = Float64()
        x_msg.data = self.target_x
        self.pub_x_reference.publish(x_msg)

        y_msg = Float64()
        y_msg.data = self.target_y
        self.pub_y_reference.publish(y_msg)

    def _publish_feedback_inputs(self):
        if not self._have_current:
            return
        x_msg = Float64()
        x_msg.data = self.current_x
        self.pub_x_feedback.publish(x_msg)

        y_msg = Float64()
        y_msg.data = self.current_y
        self.pub_y_feedback.publish(y_msg)

    def _maybe_publish_wrench(self):
        if not self._have_current:
            return
        if not (self._have_x_force or self._have_y_force):
            return

        fx_world = self._x_force_world if self._have_x_force else 0.0
        fy_world = self._y_force_world if self._have_y_force else 0.0
        fx_body, fy_body = self._rotate_world_to_body(fx_world, fy_world)

        w = Wrench()
        w.force.x = float(fx_body)
        w.force.y = float(fy_body)
        w.force.z = 0.0
        w.torque.x = 0.0
        w.torque.y = 0.0
        w.torque.z = 0.0
        self.pub_wrench.publish(w)

    def _rotate_world_to_body(self, fx_world: float, fy_world: float):
        if self._have_orientation:
            cos_yaw = math.cos(-self._yaw)
            sin_yaw = math.sin(-self._yaw)
            fx = fx_world * cos_yaw - fy_world * sin_yaw
            fy = fx_world * sin_yaw + fy_world * cos_yaw
        else:
            fx = fx_world
            fy = fy_world
        return fx, fy


def main(args=None):
    rclpy.init(args=args)

    bridge_node = BottomCameraPIDBridgeNode()
    rclpy.spin(bridge_node)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
