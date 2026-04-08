import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64

import numpy as np


class WrenchToIndividualThrusterOutputForcesNode(Node):

    def __init__(self):
        super().__init__('wrench_to_individual_thrusters_output_forces_node', namespace="orca_auv")

        self.__set_output_force_subscribers = \
            self.create_subscription(
                msg_type=Wrench,
                topic="set_output_wrench_at_center_N_Nm",
                callback=self.__set_output_wrench_at_center_subscribers_callback,
                qos_profile=10
            )

        self.__set_output_force_publishers = [
            self.create_publisher(
                msg_type=Float64,
                topic=f"thruster_{thruster_number}/set_output_force_N",
                qos_profile=10
            )
            for thruster_number in range(8)
        ]

        self.__output_force_allocation_matrix = self.__create_output_force_allocation_matrix()

    def __create_output_force_allocation_matrix(self):
        # Referencing https://hackmd.io/@NCTU-auv/HkBgyB4a3

        thrusters_profile = [
            {"position_m": np.array([ 0.127_11, -0.251_44, 0.000]), "direction": np.array([0, 0, 1])}, # NO. 0
            {"position_m": np.array([ 0.127_11,  0.251_44, 0.000]), "direction": np.array([0, 0, 1])}, # NO. 1
            {"position_m": np.array([-0.127_11, -0.251_44, 0.000]), "direction": np.array([0, 0, 1])}, # NO. 2
            {"position_m": np.array([-0.127_11,  0.251_44, 0.000]), "direction": np.array([0, 0, 1])}, # NO. 3
            {"position_m": np.array([ 0.320_49, -0.238_3, 0.000]), "direction": np.array([np.sin(np.pi / 4),  np.cos(np.pi / 4), 0])}, # NO. 4
            {"position_m": np.array([ 0.320_49,  0.238_3, 0.000]), "direction": np.array([np.sin(np.pi / 4), -np.cos(np.pi / 4), 0])}, # NO. 5
            {"position_m": np.array([-0.320_49, -0.238_3, 0.000]), "direction": np.array([np.sin(np.pi / 4), -np.cos(np.pi / 4), 0])}, # NO. 6
            {"position_m": np.array([-0.320_49,  0.238_3, 0.000]), "direction": np.array([np.sin(np.pi / 4),  np.cos(np.pi / 4), 0])}, # NO. 7
        ]

        inverse_allocation_matrix = np.vstack((
            np.column_stack(tuple(thrusters_profile[thruster_number]["direction"] for thruster_number in range(8))),
            np.column_stack(
                tuple(np.cross(thrusters_profile[thruster_number]["position_m"], thrusters_profile[thruster_number]["direction"])
                for thruster_number in range(8))),
        ))

        output_force_allocation_matrix = np.linalg.pinv(inverse_allocation_matrix)

        return output_force_allocation_matrix

    def __set_thruster_output_force(self, thruster_number, output_force_N):
        set_output_force_N = Float64()
        set_output_force_N.data = output_force_N
        self.__set_output_force_publishers[thruster_number].publish(set_output_force_N)

    def __set_output_wrench_at_center_subscribers_callback(self, msg):
        wrench_N_Nm = np.array([msg.force.x,
                                msg.force.y,
                                msg.force.z,
                                msg.torque.x,
                                msg.torque.y,
                                msg.torque.z])

        output_forces_N = self.__output_force_allocation_matrix @ wrench_N_Nm

        for thruster_number in range(8):
            self.__set_thruster_output_force(thruster_number, output_forces_N[thruster_number])


def main(args=None):
    rclpy.init(args=args)

    wrench_to_individual_thrusters_output_forces_node = WrenchToIndividualThrusterOutputForcesNode()

    rclpy.spin(wrench_to_individual_thrusters_output_forces_node)

    wrench_to_individual_thrusters_output_forces_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
