import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu


class ENUToNEDNode(Node):

    def __init__(self):
        super().__init__('enu_to_ned_node', namespace="orca_auv")
        self.__mavros_imu_data_subscriber = self.create_subscription(
            msg_type=Imu,
            topic="/mavros/imu/data",
            callback=self.__mavros_imu_data_subscriber_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            ),
        )

        self.__mavros_imu_data_subscriber = self.create_publisher(
            msg_type=Imu,
            topic="/mavros/imu/data_ned",
            qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            ),
        )

    def __mavros_imu_data_subscriber_callback(self, mavros_imu_data_msg):
        mavros_imu_data_ned_msg = Imu()

        mavros_imu_data_ned_msg.header.stamp = mavros_imu_data_msg.header.stamp
        mavros_imu_data_ned_msg.header.frame_id = mavros_imu_data_msg.header.frame_id + "_ned"

        mavros_imu_data_ned_msg.orientation = mavros_imu_data_msg.orientation
        mavros_imu_data_ned_msg.orientation_covariance = mavros_imu_data_msg.orientation_covariance

        mavros_imu_data_ned_msg.angular_velocity = mavros_imu_data_msg.angular_velocity
        mavros_imu_data_ned_msg.angular_velocity_covariance = mavros_imu_data_msg.angular_velocity_covariance

        mavros_imu_data_ned_msg.linear_acceleration = mavros_imu_data_msg.linear_acceleration
        mavros_imu_data_ned_msg.linear_acceleration_covariance = mavros_imu_data_msg.linear_acceleration_covariance


def main(args=None):
    rclpy.init(args=args)

    enu_to_ned_node = ENUToNEDNode()

    rclpy.spin(enu_to_ned_node)

    enu_to_ned_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()