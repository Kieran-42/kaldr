from setuptools import find_packages, setup

package_name = 'manual_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_nav_stack.launch.py']),
        (
            'share/' + package_name + '/config',
            ['config/nav2_params.yaml', 'config/zed_nav_override.yaml'],
        ),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='mecha-kaldr',
    maintainer_email='mecha-kaldr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_serial_bridge = manual_control.cmd_vel_serial_bridge:main',
            'move_forward_odom = manual_control.move_forward_odom:main',
        ],
    },
)
