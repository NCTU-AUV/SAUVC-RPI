from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='Robot namespace',
        ),
        Node(
            package='orca_rpi_thrusters',
            namespace=namespace,
            executable='thruster_initialization_node'
        ),
        Node(
            package='orca_rpi_thrusters',
            namespace=namespace,
            executable='thruster_force_to_pwm_output_signal_node',
            parameters=[{
                'max_output_force_N': 15.0,  # clamp thruster output before PWM conversion (expects double)
                'set_pwm_output_signal_value_max_publish_rate_hz': 20.0,  # 0 disables throttle
            }]
        ),
        Node(
            package='orca_rpi_thrusters',
            namespace=namespace,
            executable='wrench_to_individual_thrusters_output_forces_node'
        ),
    ])
