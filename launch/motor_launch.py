#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'run_test_node',
            default_value='false',
            description='Whether to run the test node alongside the motor controller'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the nodes'
        ),
        
        # Log start message
        LogInfo(
            msg='Starting DC Motor Controller...'
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
        
        # Optional test node (only if run_test_node is true)
        Node(
            package='dc_motor_controller',
            executable='motor_test_node',
            name='motor_test_node',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('run_test_node')
            ),
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])


# Alternative simple launch without conditions
def generate_simple_launch_description():
    return LaunchDescription([
        # Motor controller node
        Node(
            package='dc_motor_controller',
            executable='motor_controller_node',
            name='dc_motor_controller',
            output='screen',
            respawn=True,
        ),
        
        # Test node (commented out by default)
        # Node(
        #     package='dc_motor_controller',
        #     executable='motor_test_node',
        #     name='motor_test_node',
        #     output='screen',
        # ),
    ]) 