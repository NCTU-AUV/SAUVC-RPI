from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bottom_camera_pkg',
            executable='bottom_camera_node',
            name='bottom_camera_node',
        ),
        Node(
            package='bottom_camera_pkg',
            executable='frame_transform_node',
            name='frame_transform_node',
        ),
        Node(
            package='bottom_camera_pkg',
            executable='total_transform_node',
            name='total_transform_node',
        ),
    ])
