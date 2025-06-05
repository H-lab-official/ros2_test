#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time


class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        
        # Publishers for motor commands
        self.speed_publisher = self.create_publisher(
            Float32,
            'motor/speed',
            10)
        
        self.direction_publisher = self.create_publisher(
            Bool,
            'motor/direction',
            10)
            
        self.twist_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        # Subscribers for motor status
        self.speed_status_subscription = self.create_subscription(
            Float32,
            'motor/status/speed',
            self.speed_status_callback,
            10)
        
        self.direction_status_subscription = self.create_subscription(
            Bool,
            'motor/status/direction',
            self.direction_status_callback,
            10)
        
        # Test sequence timer
        self.test_timer = self.create_timer(3.0, self.run_test_sequence)
        self.test_step = 0
        
        self.get_logger().info('Motor Test Node Started')
        self.get_logger().info('Running automated test sequence every 3 seconds...')
        
    def speed_status_callback(self, msg):
        """Receive motor speed status"""
        self.get_logger().info(f'Motor Speed Status: {msg.data}%')
        
    def direction_status_callback(self, msg):
        """Receive motor direction status"""
        direction_str = "Forward" if msg.data else "Backward"
        self.get_logger().info(f'Motor Direction Status: {direction_str}')
        
    def run_test_sequence(self):
        """Run automated test sequence"""
        self.get_logger().info(f'Running test step {self.test_step + 1}')
        
        if self.test_step == 0:
            # Test 1: Set speed to 30% forward
            self.send_speed_command(30.0)
            self.send_direction_command(True)
            
        elif self.test_step == 1:
            # Test 2: Set speed to 60% forward
            self.send_speed_command(60.0)
            
        elif self.test_step == 2:
            # Test 3: Change direction to backward
            self.send_direction_command(False)
            
        elif self.test_step == 3:
            # Test 4: Stop motor
            self.send_speed_command(0.0)
            
        elif self.test_step == 4:
            # Test 5: Test with cmd_vel (forward)
            self.send_twist_command(0.5, 0.0)
            
        elif self.test_step == 5:
            # Test 6: Test with cmd_vel (backward)
            self.send_twist_command(-0.8, 0.0)
            
        elif self.test_step == 6:
            # Test 7: Stop with cmd_vel
            self.send_twist_command(0.0, 0.0)
            
        # Reset test sequence
        self.test_step = (self.test_step + 1) % 7
        
    def send_speed_command(self, speed):
        """Send speed command"""
        msg = Float32()
        msg.data = speed
        self.speed_publisher.publish(msg)
        self.get_logger().info(f'Sent speed command: {speed}%')
        
    def send_direction_command(self, forward):
        """Send direction command"""
        msg = Bool()
        msg.data = forward
        self.direction_publisher.publish(msg)
        direction_str = "Forward" if forward else "Backward"
        self.get_logger().info(f'Sent direction command: {direction_str}')
        
    def send_twist_command(self, linear_x, angular_z):
        """Send Twist command"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.twist_publisher.publish(msg)
        self.get_logger().info(f'Sent twist command: linear.x={linear_x}, angular.z={angular_z}')
        
    def send_manual_commands(self):
        """Manual testing functions - call these individually"""
        
        # Example manual commands:
        
        # Forward at 50%
        self.send_speed_command(50.0)
        self.send_direction_command(True)
        
        # Backward at 75%
        # self.send_speed_command(75.0)
        # self.send_direction_command(False)
        
        # Stop
        # self.send_speed_command(0.0)
        
        # Using cmd_vel
        # self.send_twist_command(1.0, 0.0)  # Full forward
        # self.send_twist_command(-0.5, 0.0)  # Half backward
        # self.send_twist_command(0.0, 0.0)  # Stop


def main(args=None):
    rclpy.init(args=args)
    
    motor_test = MotorTestNode()
    
    try:
        rclpy.spin(motor_test)
    except KeyboardInterrupt:
        motor_test.get_logger().info('Shutting down motor test node...')
    finally:
        motor_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 