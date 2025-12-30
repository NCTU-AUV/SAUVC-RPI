from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orca_auv_thruster_pkg',
            namespace='orca_auv',
            executable='thruster_initialization_node'
        ),
        Node(
            package='orca_auv_thruster_pkg',
            namespace='orca_auv',
            executable='thruster_force_to_pwm_output_signal_node',
            parameters=[{
                'max_output_force_N': 15.0,  # clamp thruster output before PWM conversion (expects double)
            }]
        ),
        Node(
            package='orca_auv_thruster_pkg',
            namespace='orca_auv',
            executable='wrench_to_individual_thrusters_output_forces_node'
        ),
    ])
