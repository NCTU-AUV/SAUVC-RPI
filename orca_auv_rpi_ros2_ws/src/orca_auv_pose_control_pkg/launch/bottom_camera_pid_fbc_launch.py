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
                    FindPackageShare("bottom_camera_pkg"),
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
        package='orca_auv_pose_control_pkg',
        namespace='orca_auv',
        executable='generic_pid_controller_node',
        name='x_coordinate_pid_controller_node',
        remappings=[
            ('/orca_auv/reference_input', '/orca_auv/bottom_camera/x_reference_input'),
            ('/orca_auv/output_feedback', '/orca_auv/bottom_camera/x_output_feedback'),
            ('/orca_auv/manipulated_variable', '/orca_auv/bottom_camera/x_force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    y_coordinate_pid_controller_node = Node(
        package='orca_auv_pose_control_pkg',
        namespace='orca_auv',
        executable='generic_pid_controller_node',
        name='y_coordinate_pid_controller_node',
        remappings=[
            ('/orca_auv/reference_input', '/orca_auv/bottom_camera/y_reference_input'),
            ('/orca_auv/output_feedback', '/orca_auv/bottom_camera/y_output_feedback'),
            ('/orca_auv/manipulated_variable', '/orca_auv/bottom_camera/y_force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    bridge_node = Node(
        package='orca_auv_pose_control_pkg',
        executable='bottom_camera_pid_bridge_node',
        parameters=[{
            'current_topic': '/orca_auv/bottom_camera/total_transform_world',
            'target_topic': '/orca_auv/target_point_px',
            'output_topic': '/camera_ctr_wrench',
            'yaw_index': 2,
            'x_reference_topic': '/orca_auv/bottom_camera/x_reference_input',
            'y_reference_topic': '/orca_auv/bottom_camera/y_reference_input',
            'x_feedback_topic': '/orca_auv/bottom_camera/x_output_feedback',
            'y_feedback_topic': '/orca_auv/bottom_camera/y_output_feedback',
            'x_manipulated_topic': '/orca_auv/bottom_camera/x_force_world_N',
            'y_manipulated_topic': '/orca_auv/bottom_camera/y_force_world_N',
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
