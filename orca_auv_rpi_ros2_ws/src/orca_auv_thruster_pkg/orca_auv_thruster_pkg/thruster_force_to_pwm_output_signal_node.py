import csv
from pathlib import Path

import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64, Int32MultiArray

import orca_auv_thruster_pkg


class ThrusterForceToPWMOutputSignalNode(Node):

    STANDARD_GRAVITY_M_PER_S_SQUARED = 9.80665
    DEFAULT_MAX_OUTPUT_FORCE_N = 5.25 * 9.80665  # match thruster_lookup_table_16V.csv max

    def __init__(self):
        super().__init__("thruster_force_to_pwm_output_signal_node", namespace="orca_auv")

        self._thruster_count = 8
        initial_pwm_us = int(self.declare_parameter("initial_pwm_output_signal_value_us", 1500).value)
        self._pwm_output_signal_value_us = [initial_pwm_us for _ in range(self._thruster_count)]

        self._max_output_force_N = float(self.declare_parameter(
            "max_output_force_N",
            self.DEFAULT_MAX_OUTPUT_FORCE_N
        ).value)
        self._positive_force_bounds_N, self._negative_force_bounds_N, self._positive_poly_coeffs, self._negative_poly_coeffs, \
            self._deadband_force_threshold_N = \
            self._fit_force_to_pwm_polynomials()

        self._pwm_output_signal_value_array_publisher = self.create_publisher(
            msg_type=Int32MultiArray,
            topic="thrusters/set_pwm_output_signal_value_us",
            qos_profile=10
        )

        self.__set_output_force_subscribers = [
            self.create_subscription(
                msg_type=Float64,
                topic=f"thruster_{thruster_number}/set_output_force_N",
                callback=lambda msg, thruster_number=thruster_number: self.__set_output_force_subscribers_callback(msg, thruster_number),
                qos_profile=10
            )
            for thruster_number in range(self._thruster_count)
        ]

    def __get_pwm_output_signal_value_us(self, output_force_N):
        if abs(output_force_N) <= self._deadband_force_threshold_N:
            return 1500

        if output_force_N >= 0:
            min_force, max_force = self._positive_force_bounds_N
            clamped_force = float(np.clip(output_force_N, min_force, max_force))
            return np.polyval(self._positive_poly_coeffs, clamped_force)

        min_force, max_force = self._negative_force_bounds_N
        clamped_force = float(np.clip(output_force_N, min_force, max_force))
        return np.polyval(self._negative_poly_coeffs, clamped_force)

    def __clamp_output_force(self, output_force_N):
        if self._max_output_force_N <= 0:
            return output_force_N

        return min(self._max_output_force_N, max(-self._max_output_force_N, output_force_N))

    def __set_output_force_subscribers_callback(self, msg, thruster_number):
        output_force_N = msg.data
        clamped_output_force_N = self.__clamp_output_force(output_force_N)
        pwm_output_signal_value_us = self.__get_pwm_output_signal_value_us(clamped_output_force_N)

        self._pwm_output_signal_value_us[thruster_number] = int(pwm_output_signal_value_us)
        self._publish_pwm_output_signal_value_array()

    def _publish_pwm_output_signal_value_array(self):
        pwm_array_msg = Int32MultiArray()
        pwm_array_msg.data = list(self._pwm_output_signal_value_us)
        self._pwm_output_signal_value_array_publisher.publish(pwm_array_msg)

    def _fit_force_to_pwm_polynomials(self):
        lookup_table_path = Path(orca_auv_thruster_pkg.__file__).parent / "thruster_lookup_table_16V.csv"
        positive_samples = []
        negative_samples = []
        min_positive_force = None
        max_negative_force = None  # closest to zero (largest negative)

        with lookup_table_path.open() as lookup_table_file:
            reader = csv.DictReader(lookup_table_file)
            for row in reader:
                pwm_us = float(row["PWM_us"])
                force_N = float(row["Force_Kg_f"]) * self.STANDARD_GRAVITY_M_PER_S_SQUARED
                if abs(force_N) < 1e-6:
                    continue
                if force_N > 0:
                    positive_samples.append((force_N, pwm_us))
                    min_positive_force = min(force_N, min_positive_force) if min_positive_force is not None else force_N
                else:
                    negative_samples.append((force_N, pwm_us))
                    max_negative_force = max(force_N, max_negative_force) if max_negative_force is not None else force_N

        positive_forces, positive_pwms = zip(*positive_samples)
        negative_forces, negative_pwms = zip(*negative_samples)

        positive_poly_coeffs = np.polyfit(positive_forces, positive_pwms, deg=3)
        negative_poly_coeffs = np.polyfit(negative_forces, negative_pwms, deg=3)

        positive_force_bounds = (min(positive_forces), max(positive_forces))
        negative_force_bounds = (min(negative_forces), max(negative_forces))

        # Use the smallest non-zero force magnitude from the table as the deadband threshold.
        positive_threshold = min_positive_force if min_positive_force is not None else float("inf")
        negative_threshold = abs(max_negative_force) if max_negative_force is not None else float("inf")
        deadband_force_threshold = min(positive_threshold, negative_threshold)
        if not np.isfinite(deadband_force_threshold):
            deadband_force_threshold = 0.0

        return positive_force_bounds, negative_force_bounds, positive_poly_coeffs, negative_poly_coeffs, deadband_force_threshold


def main(args=None):
    rclpy.init(args=args)

    thruster_force_to_pwm_output_signal_node = ThrusterForceToPWMOutputSignalNode()
    rclpy.spin(thruster_force_to_pwm_output_signal_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
