from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    thruster_pkg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('thrusters'),
            'launch',
            'thrusters.launch.py'
        ])),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    wrench_sum_node = Node(
        package='wrench_sum',
        executable='wrench_sum_node',
        namespace=namespace,
        name='wrench_sum_node',
        parameters=[{
            'input_topics': [
                'control/wrench_sources/gui',
                'control/wrench_sources/bottom_camera',
                'control/wrench_sources/depth',
            ],
            'output_topic': 'control/wrench_command',
        }]
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[{'fcu_url': 'serial:///dev/ttyACM0:2000000'}]
    )

    gui_node = Node(
        package='gui',
        executable='gui_node',
        namespace=namespace,
        remappings=[
            ('control/wrench_command', 'control/wrench_sources/gui')
        ]
    )

    supervisor_node = Node(
        package='system_manager',
        executable='supervisor_node',
        namespace=namespace,
        name='supervisor_node',
    )

    bottom_camera_pid_fbc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xy_translation_control'),
                'launch',
                'bottom_camera_pid_fbc_launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    depth_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depth_control'),
                'launch',
                'depth_control_launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0']
    )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
    )

    stm32_flasher_node = Node(
        package='stm32_manager',
        executable='stm32_flasher_node',
        namespace=namespace,
        name='stm32_flasher_node',
    )

    flash_stm32 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/flash_stm32', 'std_srvs/srv/Trigger', '{}'],
        shell=True,
    )

    event = RegisterEventHandler(
        OnProcessStart(
            target_action=micro_ros_agent,
            on_start=[flash_stm32]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='Robot namespace',
        ),
        bottom_camera_pid_fbc_launch,
        depth_control_launch,
        thruster_pkg_launch,
        wrench_sum_node,
        # mavros,
        supervisor_node,
        gui_node,
        stm32_flasher_node,
        micro_ros_agent,
        web_video_server,
        event,
    ])
