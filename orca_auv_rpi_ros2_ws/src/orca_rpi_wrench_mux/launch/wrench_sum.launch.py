from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orca_rpi_wrench_mux',
            executable='orca_rpi_wrench_mux_node',
            name='orca_rpi_wrench_mux',
            output='screen',
            parameters=[{
                # 在這裡列出你所有需要整合的 topic
                'input_topics': [
                    '/auv/thruster_1/wrench',
                    '/auv/thruster_2/wrench',
                    '/auv/external_force/wrench'
                ],
                'output_topic': '/auv/total_wrench',
                'publish_rate': 30.0
            }]
        )
    ])
