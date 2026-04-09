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
            )
        )
    )

    on_off_controller_node = Node(
        package="orca_rpi_control",
        executable="on_off_controller",
        parameters=[
            {"current_topic": "/orca_auv/bottom_camera/total_transform_world"},
            {"target_topic": "/orca_auv/target_point_px"},
            {"output_topic": "/camera_ctr_wrench"},
            {"tol_x": 5.0},
            {"tol_y": 5.0},
            {"thrust": 1.0},
            {"single_axis_only": False},
        ],
    )

    return LaunchDescription(
        [
            bottom_camera_launch,
            on_off_controller_node,
        ]
    )
