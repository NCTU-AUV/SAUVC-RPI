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

    wrench_sum_node = Node(
        package='wrench_sum',
        executable='wrench_sum_node',
        name='wrench_sum',
        parameters=[{
            'input_topics': [
                '/gui_wrench',
                '/camera_ctr_wrench',
                '/depth_ctr_wrench',
                "/velocity_ctr_wrench",
            ],
            'output_topic': '/orca_auv/set_output_wrench_at_center_N_Nm',
        }]
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[{'fcu_url': 'serial:///dev/ttyACM0:2000000'}]
    )

    gui_node = Node(
        package='gui_pkg',
        executable='gui_node',
        remappings=[
            ('/orca_auv/set_output_wrench_at_center_N_Nm', '/gui_wrench')
        ]
    )
    velocity_node = Node(
        package='orca_auv_pose_control_pkg',
        executable='velocity_controller_node',
        name='velocity_controller_node',
        parameters=[{
            'measured_topic': '/orca_auv/bottom_camera/velocity_mps',
            'measured_index': 1,
            'target_topic': '/orca_auv/target_speed_mps',
            'output_topic': '/velocity_ctr_wrench',

            'kp': 8.0,
            'ki': 0.0,
            'kd': 0.0,
            'd_cutoff_hz': 5.0,

            'max_force_N': 25.0,
            'min_force_N': -25.0,
            'i_limit': 10.0,
            'deadband_mps': 0.01,
            'control_rate_hz': 30.0,
            'measurement_timeout_s': 0.5,
        }]
    )

    bottom_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bottom_camera_pkg'),
                'launch',
                'bottom_camera_optical_flow.launch.py'
            ])
        ),
        launch_arguments={
            'publish_debug_image': 'true'
        }.items()
    )

    bottom_camera_pid_fbc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orca_auv_pose_control_pkg'),
                'launch',
                'bottom_camera_pid_fbc_launch.py'
            ])
        )
    )

    depth_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orca_auv_pose_control_pkg'),
                'launch',
                'depth_control_launch.py'
            ])
        )
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0']
    )

    stm32_flasher_node = Node(
        package='stm32_pkg',
        executable='stm32_flasher_node',
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
        bottom_camera_launch,
        bottom_camera_pid_fbc_launch,
        depth_control_launch,
        thruster_pkg_launch,
        wrench_sum_node,
        # velocity_node,
        # mavros,
        gui_node,
        stm32_flasher_node,
        micro_ros_agent,
        event,
    ])
