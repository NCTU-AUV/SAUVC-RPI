from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    thruster_pkg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orca_auv_thruster_pkg'),
            'launch',
            'start_all_nodes_launch.py'
        ])),
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[{'fcu_url': 'serial:///dev/ttyACM0:2000000'}]
    )

    gui_node = Node(
        package='gui_pkg',
        executable='gui_node',
    )

    bottom_camera_node = Node(
        package='bottom_camera_pkg',
        executable='bottom_camera_node',
        parameters=[
            {'camera_device': '/dev/video0'}
        ]
    )

    frame_transform_node = Node(
        package='bottom_camera_pkg',
        executable='frame_transform_node',
        parameters=[
            {'image_topic': 'bottom_camera/image_raw'}
        ]
    )
    waypoint_target_publisher = Node(
        package='orca_auv_pose_control_pkg',
        executable='waypoint_target_publisher',
        parameters=[
            {'current_topic': '/orca_auv/bottom_camera/total_transform_world'},
            {'target_topic': '/orca_auv/target_point_px'},
            {'done_topic': '/orca_auv/target_done'},
            {'tol_x': 5.0},
            {'tol_y': 5.0},
            {'stable_count': 5},
            {'publish_first_immediately': True},
        ]
    )

    on_off_controller = Node(
        package='orca_auv_pose_control_pkg',
        executable='on_off_controller',
        parameters=[
            {'current_topic': '/orca_auv/bottom_camera/total_transform_world'},
            {'target_topic': '/orca_auv/target_point_px'},
            {'output_topic': '/orca_auv/set_output_wrench_at_center_N_Nm'},
            {'tol_x': 5.0},
            {'tol_y': 5.0},
            {'thrust': 10.0},
            {'single_axis_only': True},
        ]
    )


    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0']
    )

    flash_stm32 = ExecuteProcess(
        cmd=['st-flash', '--reset', 'write', '/root/orca_auv_rpi_ros2_ws/stm32_binary/SAUVC2024.bin', '0x08000000'],
        shell=True,
    )

    event = RegisterEventHandler(
        OnProcessStart(
            target_action=micro_ros_agent,
            on_start=[flash_stm32]
        )
    )

    return LaunchDescription([
        thruster_pkg_launch,
        mavros,
        gui_node,
        bottom_camera_node,
        frame_transform_node,
        waypoint_target_publisher,
        on_off_controller,
        micro_ros_agent,
        event,
    ])
