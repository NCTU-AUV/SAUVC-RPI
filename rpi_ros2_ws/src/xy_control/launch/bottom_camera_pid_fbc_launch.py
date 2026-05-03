from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    x_coordinate_pid_controller_node = Node(
        package='control',
        namespace=namespace,
        executable='generic_pid_controller_node',
        name='x_coordinate_pid_controller_node',
        remappings=[
            ('control/pid/reference', 'control/pid/bottom_camera/x/reference_px'),
            ('control/pid/feedback', 'control/pid/bottom_camera/x/feedback_px'),
            ('control/pid/output', 'control/pid/bottom_camera/x/force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    y_coordinate_pid_controller_node = Node(
        package='control',
        namespace=namespace,
        executable='generic_pid_controller_node',
        name='y_coordinate_pid_controller_node',
        remappings=[
            ('control/pid/reference', 'control/pid/bottom_camera/y/reference_px'),
            ('control/pid/feedback', 'control/pid/bottom_camera/y/feedback_px'),
            ('control/pid/output', 'control/pid/bottom_camera/y/force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    bridge_node = Node(
        package='xy_control',
        executable='bottom_camera_pid_bridge_node',
        namespace=namespace,
        parameters=[{
            'yaw_topic': 'state/bottom_camera/yaw_rad',
            'output_topic': 'control/wrench_sources/bottom_camera',
            'x_manipulated_topic': 'control/pid/bottom_camera/x/force_world_N',
            'y_manipulated_topic': 'control/pid/bottom_camera/y/force_world_N',
        }],
    )

    waypoint_target_publisher = Node(
        package='xy_control',
        executable='waypoint_target_publisher',
        namespace=namespace,
        name='waypoint_target_publisher',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='orca_auv',
                description='Robot namespace',
            ),
            x_coordinate_pid_controller_node,
            y_coordinate_pid_controller_node,
            bridge_node,
            waypoint_target_publisher,
        ]
    )
