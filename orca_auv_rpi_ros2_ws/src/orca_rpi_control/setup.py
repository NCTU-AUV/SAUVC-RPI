import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'orca_rpi_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Only install launch scripts, avoid copying __pycache__ directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            "depth_controller_node = orca_rpi_control.depth_controller_node:main",
            "generic_pid_controller_node = orca_rpi_control.generic_pid_controller_node:main",
            "float32_to_float64_converter_node = orca_rpi_control.float32_to_float64_converter_node:main",
            "output_sink_force_to_output_wrench_node = orca_rpi_control.output_sink_force_to_output_wrench_node:main",
            "imu_to_orientation_node = orca_rpi_control.imu_to_orientation_node:main",
            "on_off_controller = orca_rpi_control.on_off_controller:main",
            "bottom_camera_pid_bridge_node = orca_rpi_control.bottom_camera_pid_bridge_node:main",
            "waypoint_target_publisher = orca_rpi_control.waypoint_target_publisher:main",
            "velocity_controller_node = orca_rpi_control.velocity_controller_node:main",
        ],
    },
)
