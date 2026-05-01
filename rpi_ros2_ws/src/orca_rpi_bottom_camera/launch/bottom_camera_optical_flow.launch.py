from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Whether to publish debug keypoint overlay images'
        ),
        Node(
            package='orca_rpi_bottom_camera',
            executable='bottom_camera_node',
            namespace=namespace,
            name='bottom_camera_node',
        ),
        Node(
            package='orca_rpi_bottom_camera',
            executable='lk_total_transform_node',
            namespace=namespace,
            name='lk_total_transform_node',
            parameters=[{'publish_debug_image': publish_debug_image}],
        ),
    ])
