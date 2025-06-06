#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
from threading import Thread


class DCMotorController(Node):
    def __init__(self):
        super().__init__('dc_motor_controller')
        
        # GPIO Pin Configuration (BCM numbering)
        self.PWM_PIN = 12   # Physical pin 32 = GPIO 12 (PWM0)
        self.DIR_PIN = 13   # Physical pin 33 = GPIO 13 (PWM1)
        # Ground pin 25 จะต่อกับ GND ของมอเตอร์
        
        # PWM Parameters
        self.PWM_FREQUENCY = 1000  # 1kHz
        self.current_speed = 0.0
        self.current_direction = True  # True = Forward, False = Backward
        self.motor_enabled = False  # มอเตอร์จะหมุนเฉพาะเมื่อได้รับคำสั่ง
        
        # Initialize GPIO
        self.setup_gpio()
        
        # ROS2 Subscriptions
        self.speed_subscription = self.create_subscription(
            Float32,
            'motor/speed',
            self.speed_callback,
            0)
        
        self.direction_subscription = self.create_subscription(
            Bool,
            'motor/direction',
            self.direction_callback,
            0)
            
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            0)
        
        # ROS2 Publishers for status
        self.status_publisher = self.create_publisher(
            Float32,
            'motor/status/speed',
            0)
        
        self.direction_status_publisher = self.create_publisher(
            Bool,
            'motor/status/direction',
            0)
        
        # Status publishing timer
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('DC Motor Controller Node Started')
        self.get_logger().info(f'PWM Pin: {self.PWM_PIN}, Direction Pin: {self.DIR_PIN}')
        
    def setup_gpio(self):
        """Initialize GPIO pins for motor control"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup direction pin
            GPIO.setup(self.DIR_PIN, GPIO.OUT)
            GPIO.output(self.DIR_PIN, GPIO.LOW)  # เริ่มต้นด้วยการหยุดมอเตอร์
            
            # Setup PWM pin
            GPIO.setup(self.PWM_PIN, GPIO.OUT)
            self.pwm = GPIO.PWM(self.PWM_PIN, self.PWM_FREQUENCY)
            self.pwm.start(0)  # Start with 0% duty cycle
            
            self.get_logger().info('GPIO initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {str(e)}')
            
    def speed_callback(self, msg):
        """Handle speed commands (0.0 to 100.0)"""
        speed = max(0.0, min(100.0, msg.data))  # Clamp between 0-100
        self.current_speed = speed
        
        # เปิดใช้งานมอเตอร์เฉพาะเมื่อมีความเร็ว > 0
        self.motor_enabled = (speed > 0.0)
        
        self.update_motor()
        self.get_logger().info(f'Received speed command: {speed}%')
        
    def direction_callback(self, msg):
        """Handle direction commands (True=Forward, False=Backward)"""
        self.current_direction = msg.data
        
        # เปิดใช้งานมอเตอร์เมื่อได้รับคำสั่งทิศทาง และมีความเร็ว > 0
        if self.current_speed > 0.0:
            self.motor_enabled = True
        
        self.update_motor()
        direction_str = "Forward" if msg.data else "Backward"
        self.get_logger().info(f'Received direction command: {direction_str}')
        
    def twist_callback(self, msg):
        """Handle Twist messages from cmd_vel topic"""
        # Convert linear.x velocity to speed percentage
        # Assuming max velocity of 1.0 m/s corresponds to 100% speed
        linear_x = msg.linear.x
        
        # แก้ไข: คำนวณความเร็วและทิศทางจาก linear_x อย่างถูกต้อง
        if linear_x >= 0:
            self.current_direction = True  # Forward
            self.current_speed = min(100.0, linear_x * 100.0)  # แปลงกลับเป็น % โดยไม่ใช้ abs()
        else:
            self.current_direction = False  # Backward
            self.current_speed = min(100.0, abs(linear_x) * 100.0)  # ใช้ abs() เฉพาะกับค่าลบ
        
        # เปิดใช้งานมอเตอร์เฉพาะเมื่อมีความเร็ว > 0
        self.motor_enabled = (self.current_speed > 0.0)
            
        self.update_motor()
        self.get_logger().info(f'Twist command - Speed: {self.current_speed}%, Direction: {"Forward" if self.current_direction else "Backward"}')
        
    def update_motor(self):
        """Update motor PWM and direction based on current settings"""
        try:
            if self.motor_enabled and self.current_speed > 0.0:
                # Set direction เฉพาะเมื่อมอเตอร์ทำงาน
                GPIO.output(self.DIR_PIN, GPIO.HIGH if self.current_direction else GPIO.LOW)
                
                # Set PWM duty cycle
                self.pwm.ChangeDutyCycle(self.current_speed)
            else:
                # หยุดมอเตอร์ - ปิด PWM และ direction signal
                GPIO.output(self.DIR_PIN, GPIO.LOW)
                self.pwm.ChangeDutyCycle(0)
            
        except Exception as e:
            self.get_logger().error(f'Failed to update motor: {str(e)}')
            
    def publish_status(self):
        """Publish current motor status"""
        # Publish speed status
        speed_msg = Float32()
        speed_msg.data = self.current_speed
        self.status_publisher.publish(speed_msg)
        
        # Publish direction status
        direction_msg = Bool()
        direction_msg.data = self.current_direction
        self.direction_status_publisher.publish(direction_msg)
        
    def stop_motor(self):
        """Emergency stop function"""
        self.current_speed = 0.0
        self.motor_enabled = False
        self.update_motor()
        self.get_logger().info('Motor stopped')
        
    def cleanup_gpio(self):
        """Clean up GPIO resources"""
        try:
            self.stop_motor()
            self.pwm.stop()
            GPIO.cleanup()
            self.get_logger().info('GPIO cleaned up')
        except Exception as e:
            self.get_logger().error(f'Error during GPIO cleanup: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    motor_controller = DCMotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info('Shutting down motor controller...')
    finally:
        motor_controller.cleanup_gpio()
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 