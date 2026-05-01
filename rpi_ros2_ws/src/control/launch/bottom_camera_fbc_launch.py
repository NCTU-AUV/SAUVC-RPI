from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    bottom_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("bottom_camera"),
                    "launch",
                    # "bottom_camera.launch.py",
                    "bottom_camera_optical_flow.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    on_off_controller_node = Node(
        package="control",
        executable="on_off_controller",
        namespace=namespace,
        parameters=[
            {"current_topic": "camera/bottom/pose_px"},
            {"target_topic": "control/targets/bottom_camera_point_px"},
            {"output_topic": "control/wrench_sources/bottom_camera"},
            {"tol_x": 5.0},
            {"tol_y": 5.0},
            {"thrust": 1.0},
            {"single_axis_only": False},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='orca_auv',
                description='Robot namespace',
            ),
            bottom_camera_launch,
            on_off_controller_node,
        ]
    )
