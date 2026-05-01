from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bottom_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("orca_rpi_bottom_camera"),
                    "launch",
                    # "bottom_camera.launch.py",
                    "bottom_camera_optical_flow.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "publish_debug_image": "false",
        }.items(),
    )

    x_coordinate_pid_controller_node = Node(
        package='orca_rpi_control',
        namespace='orca_auv',
        executable='generic_pid_controller_node',
        name='x_coordinate_pid_controller_node',
        remappings=[
            ('/orca_auv/control/pid/reference', '/orca_auv/control/pid/bottom_camera/x/reference_px'),
            ('/orca_auv/control/pid/feedback', '/orca_auv/control/pid/bottom_camera/x/feedback_px'),
            ('/orca_auv/control/pid/output', '/orca_auv/control/pid/bottom_camera/x/force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    y_coordinate_pid_controller_node = Node(
        package='orca_rpi_control',
        namespace='orca_auv',
        executable='generic_pid_controller_node',
        name='y_coordinate_pid_controller_node',
        remappings=[
            ('/orca_auv/control/pid/reference', '/orca_auv/control/pid/bottom_camera/y/reference_px'),
            ('/orca_auv/control/pid/feedback', '/orca_auv/control/pid/bottom_camera/y/feedback_px'),
            ('/orca_auv/control/pid/output', '/orca_auv/control/pid/bottom_camera/y/force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    bridge_node = Node(
        package='orca_rpi_control',
        executable='bottom_camera_pid_bridge_node',
        parameters=[{
            'current_topic': '/orca_auv/camera/bottom/pose_px',
            'target_topic': '/orca_auv/control/targets/bottom_camera_point_px',
            'output_topic': '/orca_auv/control/wrench_sources/bottom_camera',
            'yaw_index': 2,
            'x_reference_topic': '/orca_auv/control/pid/bottom_camera/x/reference_px',
            'y_reference_topic': '/orca_auv/control/pid/bottom_camera/y/reference_px',
            'x_feedback_topic': '/orca_auv/control/pid/bottom_camera/x/feedback_px',
            'y_feedback_topic': '/orca_auv/control/pid/bottom_camera/y/feedback_px',
            'x_manipulated_topic': '/orca_auv/control/pid/bottom_camera/x/force_world_N',
            'y_manipulated_topic': '/orca_auv/control/pid/bottom_camera/y/force_world_N',
        }],
    )

    return LaunchDescription(
        [
            bottom_camera_launch,
            x_coordinate_pid_controller_node,
            y_coordinate_pid_controller_node,
            bridge_node,
        ]
    )
