from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='false',
            description='Whether to publish debug keypoint overlay images'
        ),
        Node(
            package='bottom_camera_pkg',
            executable='bottom_camera_node',
            name='bottom_camera_node',
        ),
        Node(
            package='bottom_camera_pkg',
            executable='frame_transform_node',
            name='frame_transform_node',
            parameters=[{'publish_debug_image': publish_debug_image}],
        ),
        Node(
            package='bottom_camera_pkg',
            executable='total_transform_node',
            name='total_transform_node',
        ),
    ])
