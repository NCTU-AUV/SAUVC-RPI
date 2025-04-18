from setuptools import find_packages, setup

package_name = 'orca_auv_thruster_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "thruster_initialization_node = orca_auv_thruster_pkg.thruster_initialization_node:main",
            "thruster_force_to_pwm_output_signal_node = orca_auv_thruster_pkg.thruster_force_to_pwm_output_signal_node:main",
            "wrench_to_individual_thrusters_output_forces_node = orca_auv_thruster_pkg.wrench_to_individual_thrusters_output_forces_node:main",
        ],
    },
)
