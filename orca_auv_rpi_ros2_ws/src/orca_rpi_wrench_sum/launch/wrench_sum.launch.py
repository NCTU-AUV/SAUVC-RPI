from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orca_rpi_wrench_sum',
            executable='orca_rpi_wrench_sum_node',
            name='orca_rpi_wrench_sum',
            output='screen',
            parameters=[{
                # 在這裡列出你所有需要整合的 topic
                'input_topics': [
                    '/orca_auv/control/wrench_sources/gui',
                    '/orca_auv/control/wrench_sources/bottom_camera',
                    '/orca_auv/control/wrench_sources/depth',
                    '/orca_auv/control/wrench_sources/velocity',
                ],
                'output_topic': '/orca_auv/control/wrench_command',
                'publish_rate': 30.0
            }]
        )
    ])
