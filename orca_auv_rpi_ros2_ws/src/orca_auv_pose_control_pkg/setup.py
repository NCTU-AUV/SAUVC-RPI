import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'orca_auv_pose_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "depth_controller_node = orca_auv_pose_control_pkg.depth_controller_node:main",
            "generic_pid_controller_node = orca_auv_pose_control_pkg.generic_pid_controller_node:main",
            "float32_float64_converter_node = orca_auv_pose_control_pkg.float32_float64_converter_node:main",
            "output_sink_force_to_output_wrench_node = orca_auv_pose_control_pkg.output_sink_force_to_output_wrench_node:main",
            "imu_to_orientation_node = orca_auv_pose_control_pkg.imu_to_orientation_node:main",
            "on_off_controller = orca_auv_pose_control_pkg.on_off_controller:main",
            "waypoint_target_publisher = orca_auv_pose_control_pkg.waypoint_target_publisher:main"
        ],
    },
)
