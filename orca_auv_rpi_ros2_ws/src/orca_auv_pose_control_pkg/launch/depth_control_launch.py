from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orca_auv_pose_control_pkg',
            namespace='orca_auv',
            executable='generic_pid_controller_node',
            name='depth_pid_controller_node',
            remappings=[
                ('/orca_auv/reference_input', '/orca_auv/target_depth_m'),
                ('/orca_auv/output_feedback', '/orca_auv/pressure_sensor_depth_float64_m'),
                ('/orca_auv/manipulated_variable', '/orca_auv/output_sink_force_N'),
            ],
            parameters=[{
                'proportional_gain': 20.0,
                'integral_gain': 2.0,
                'derivative_gain': 0.0,
                'derivative_smoothing_factor': 0.0,
            }],
        ),
        Node(
            package='orca_auv_pose_control_pkg',
            namespace='orca_auv',
            executable='output_sink_force_to_output_wrench_node'
        ),
        Node(
            package='orca_auv_pose_control_pkg',
            namespace='orca_auv',
            executable='float32_float64_converter_node',
            remappings=[
                ('/orca_auv/float32_topic', '/orca_auv/pressure_sensor_depth_m'),
                ('/orca_auv/float64_topic', '/orca_auv/pressure_sensor_depth_float64_m'),
            ],
        ),
    ])
