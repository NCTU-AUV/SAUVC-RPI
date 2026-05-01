from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orca_rpi_control',
            namespace='orca_auv',
            executable='generic_pid_controller_node',
            name='depth_pid_controller_node',
            remappings=[
                ('/orca_auv/control/pid/reference', '/orca_auv/control/targets/depth_m'),
                ('/orca_auv/control/pid/feedback', '/orca_auv/state/depth_m'),
                ('/orca_auv/control/pid/output', '/orca_auv/control/pid/depth/sink_force_N'),
            ],
            parameters=[{
                'proportional_gain': 30.0,
                'integral_gain': 2.0,
                'derivative_gain': 30.0,
                'derivative_smoothing_factor': 0.0,
            }],
        ),
        Node(
            package='orca_rpi_control',
            namespace='orca_auv',
            executable='output_sink_force_to_output_wrench_node',
            remappings=[
                ('/orca_auv/control/wrench_command', '/orca_auv/control/wrench_sources/depth'),
            ],
        ),
        Node(
            package='orca_rpi_control',
            namespace='orca_auv',
            executable='float32_to_float64_converter_node',
            remappings=[
                ('/orca_auv/converters/float32_input', '/orca_auv/sensors/depth_m'),
                ('/orca_auv/converters/float64_output', '/orca_auv/state/depth_m'),
            ],
        ),
        Node(
            package='orca_rpi_control',
            namespace='orca_auv',
            executable='imu_to_orientation_node',
        ),
    ])
