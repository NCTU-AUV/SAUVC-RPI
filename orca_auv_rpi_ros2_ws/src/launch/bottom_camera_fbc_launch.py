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
            )
        )
    )

    on_off_controller_node = Node(
        package="orca_auv_pose_control_pkg",
        executable="on_off_controller",
        parameters=[
            {"current_topic": "/orca_auv/bottom_camera/total_transform_world"},
            {"target_topic": "/orca_auv/target_point_px"},
            {"output_topic": "/orca_auv/set_output_wrench_at_center_N_Nm"},
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
