from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    thruster_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("orca_auv_thruster_pkg"),
                    "launch",
                    "start_all_nodes_launch.py",
                ]
            )
        )
    )

    bottom_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("bottom_camera_pkg"),
                    "launch",
                    "bottom_camera.launch.py",
                ]
            )
        )
    )

    gui_node = Node(
        package="gui_pkg",
        executable="gui_node",
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
            {"thrust": 0.5},
            {"single_axis_only": True},
        ],
    )

    return LaunchDescription(
        [
            thruster_control,
            bottom_camera_launch,
            gui_node,
            on_off_controller_node,
        ]
    )
