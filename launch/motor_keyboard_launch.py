#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the nodes'
        ),
        
        # Log start message
        LogInfo(
            msg='Starting DC Motor Controller with Keyboard Control...'
        ),
        
        # Main motor controller node
        Node(
            package='dc_motor_controller',
            executable='motor_controller_node',
            name='dc_motor_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            respawn=True,
            respawn_delay=2.0,
        ),
        
        # Keyboard control node
        Node(
            package='dc_motor_controller',
            executable='keyboard_control_node',
            name='keyboard_control_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            # ไม่ใส่ respawn เพราะ keyboard control จะจบเมื่อ user กด q
        ),
    ]) 