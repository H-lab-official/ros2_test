#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Motor Controller Node
        Node(
            package='dc_motor_controller',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
            parameters=[
                {'pwm_pin': 12},
                {'dir_pin': 13},
                {'pwm_frequency': 1000}
            ]
        ),
        
        # Momentary Keyboard Control Node
        Node(
            package='dc_motor_controller', 
            executable='keyboard_control_momentary',
            name='keyboard_control_momentary',
            output='screen'
        )
    ]) 