from setuptools import find_packages, setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'static/controller/*.html',
            'static/controller/*.js',
            'static/dashboard/*.html',
            'static/dashboard/*.js',
            'static/shared/*.js',
            'static/shared/*.png',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gui_node = gui.gui_node:main"
        ],
    },
)
