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
        micro_ros_agent,
        event,
    ])
