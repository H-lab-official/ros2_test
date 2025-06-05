from setuptools import setup

package_name = 'dc_motor_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_launch.py', 'launch/motor_keyboard_launch.py']),
        ('share/' + package_name + '/config', ['config/motor_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DC Motor Controller',
    maintainer_email='user@example.com',
    description='ROS2 package for controlling DC motor via PWM on Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = dc_motor_controller.motor_controller_node:main',
            'motor_test_node = dc_motor_controller.motor_test_node:main',
            'keyboard_control_node = dc_motor_controller.keyboard_control_node:main',
            'h_bridge_motor_controller = dc_motor_controller.h_bridge_motor_controller:main',
        ],
    },
    python_requires='>=3.6',
) 