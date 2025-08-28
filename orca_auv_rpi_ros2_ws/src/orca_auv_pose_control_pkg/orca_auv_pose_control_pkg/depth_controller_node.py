import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Wrench


class DepthControllerNode(Node):

    def __init__(self):
        super().__init__('depth_controller_node', namespace="orca_auv")

        self._pressure_sensor_depth_subscriber = self.create_subscription(
            Float32,
            'pressure_sensor_depth_m',
            self._pressure_sensor_depth_subscription_callback,
            10)

        self._target_depth_subscriber = self.create_subscription(
            Float32,
            'target_depth_m',
            self._target_depth_subscription_callback,
            10)

        self._set_output_wrench_at_center_publisher = self.create_publisher(Wrench, 'set_output_wrench_at_center_N_Nm', 10)

        integral_controller_loop_timer_period_s = 1 / 100
        self._integral_controller_loop_timer = self.create_timer(integral_controller_loop_timer_period_s, self._integral_controller_loop_timer_callback)

        self._previous_time = self.get_clock().now()

        self._pressure_sensor_depth_m = 0.0
        self._target_depth_m = 0.0

        self._force_on_z_direction_N = 0.0

        self._integration_factor_N_per_m_s = 2.0
        self._integrated_force_on_z_direction_N = 0.0

        self._proportional_factor_N_per_m = 20.0

    def _pressure_sensor_depth_subscription_callback(self, msg):
        self._pressure_sensor_depth_m = msg.data

    def _target_depth_subscription_callback(self, msg):
        self._target_depth_m = msg.data

    def _integral_controller_loop_timer_callback(self):
        current_time = self.get_clock().now()

        self._integrated_force_on_z_direction_N += (
            (self._target_depth_m - self._pressure_sensor_depth_m)
            * (current_time - self._previous_time).nanoseconds / (10**9)
            * self._integration_factor_N_per_m_s
        )

        self._force_on_z_direction_N = (
            (self._target_depth_m - self._pressure_sensor_depth_m)
            * self._proportional_factor_N_per_m
        ) + self._integrated_force_on_z_direction_N

        self._publish_force_on_z_direction(self._force_on_z_direction_N)

        self._previous_time = current_time

    def _publish_force_on_z_direction(self, force_on_z_direction_N):
        msg = Wrench()

        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = force_on_z_direction_N
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0

        self._set_output_wrench_at_center_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    depth_controller_node = DepthControllerNode()

    rclpy.spin(depth_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
