from setuptools import find_packages, setup
from glob import glob

package_name = 'orca_rpi_bottom_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
            'bottom_camera_node = orca_rpi_bottom_camera.bottom_camera_node:main',
            'lk_total_transform_node = orca_rpi_bottom_camera.lk_total_transform_node:main',
            # 'frame_transform_node = orca_rpi_bottom_camera.frame_transform_node:main',
            # 'total_transform_node = orca_rpi_bottom_camera.total_transform_node:main',
            'pixel_to_meter_node = orca_rpi_bottom_camera.pixel_to_meter_node:main',
            'delta_to_velocity_node = orca_rpi_bottom_camera.delta_to_velocity_node:main',
        ],
    },
)
