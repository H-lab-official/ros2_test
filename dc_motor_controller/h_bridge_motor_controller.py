#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time
from threading import Thread

# ใช้ Mock GPIO เสมอสำหรับการทดสอบ
USE_MOCK_GPIO = True

if USE_MOCK_GPIO:
    from dc_motor_controller.mock_gpio import MockGPIO as GPIO
    print("Using Mock GPIO for testing")
else:
    try:
        import RPi.GPIO as GPIO
        print("Using Real GPIO")
    except (ImportError, RuntimeError) as e:
        from dc_motor_controller.mock_gpio import MockGPIO as GPIO
        print(f"Failed to import RPi.GPIO ({e}), using Mock GPIO")


class HBridgeMotorController(Node):
    def __init__(self):
        super().__init__('h_bridge_motor_controller')
        
        # H-Bridge Pin Configuration (BCM numbering)
        # Motor A
        self.ENA_PIN = 12    # Physical pin 32 = GPIO 12 (Hardware PWM0 for Motor A)
        self.IN1_PIN = 13    # Physical pin 33 = GPIO 13 (Direction A)
        self.IN2_PIN = 19    # Physical pin 35 = GPIO 19 (Direction A)
        
        # Motor B (สำหรับอนาคต - ใช้ Hardware PWM1)
        self.ENB_PIN = 18    # Physical pin 12 = GPIO 18 (Hardware PWM1 for Motor B)
        self.IN3_PIN = 26    # Physical pin 37 = GPIO 26 (Direction B)
        self.IN4_PIN = 20    # Physical pin 38 = GPIO 20 (Direction B)
        
        # PWM Parameters
        self.PWM_FREQUENCY = 1000  # 1kHz
        self.current_speed = 0.0
        self.current_direction = True  # True = Forward, False = Backward
        self.pwm_a = None
        self.pwm_b = None
        
        # Initialize GPIO
        self.setup_gpio()
        
        # ROS2 Subscriptions
        self.speed_subscription = self.create_subscription(
            Float32,
            'motor/speed',
            self.speed_callback,
            10)
        
        self.direction_subscription = self.create_subscription(
            Bool,
            'motor/direction',
            self.direction_callback,
            10)
            
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        
        # ROS2 Publishers for status
        self.status_publisher = self.create_publisher(
            Float32,
            'motor/status/speed',
            10)
        
        self.direction_status_publisher = self.create_publisher(
            Bool,
            'motor/status/direction',
            10)
        
        # Status publishing timer
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('H-Bridge Motor Controller Node Started')
        self.get_logger().info(f'Motor A - ENA: GPIO {self.ENA_PIN}, IN1: GPIO {self.IN1_PIN}, IN2: GPIO {self.IN2_PIN}')
        
    def setup_gpio(self):
        """Initialize GPIO pins for H-Bridge motor control"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup Motor A pins
            GPIO.setup(self.ENA_PIN, GPIO.OUT)  # PWM pin
            GPIO.setup(self.IN1_PIN, GPIO.OUT)  # Direction pin 1
            GPIO.setup(self.IN2_PIN, GPIO.OUT)  # Direction pin 2
            
            # Setup Motor B pins (สำหรับอนาคต)
            GPIO.setup(self.ENB_PIN, GPIO.OUT)  # PWM pin
            GPIO.setup(self.IN3_PIN, GPIO.OUT)  # Direction pin 1
            GPIO.setup(self.IN4_PIN, GPIO.OUT)  # Direction pin 2
            
            # Initialize direction (stop)
            GPIO.output(self.IN1_PIN, GPIO.LOW)
            GPIO.output(self.IN2_PIN, GPIO.LOW)
            GPIO.output(self.IN3_PIN, GPIO.LOW)
            GPIO.output(self.IN4_PIN, GPIO.LOW)
            
            # Setup PWM
            self.pwm_a = GPIO.PWM(self.ENA_PIN, self.PWM_FREQUENCY)
            self.pwm_b = GPIO.PWM(self.ENB_PIN, self.PWM_FREQUENCY)
            self.pwm_a.start(0)  # Start with 0% duty cycle
            self.pwm_b.start(0)  # Start with 0% duty cycle
            
            self.get_logger().info('H-Bridge GPIO initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {str(e)}')
            
    def speed_callback(self, msg):
        """Handle speed commands (0.0 to 100.0)"""
        speed = max(0.0, min(100.0, msg.data))  # Clamp between 0-100
        self.current_speed = speed
        self.update_motor()
        self.get_logger().info(f'Received speed command: {speed}%')
        
    def direction_callback(self, msg):
        """Handle direction commands (True=Forward, False=Backward)"""
        self.current_direction = msg.data
        self.update_motor()
        direction_str = "Forward" if msg.data else "Backward"
        self.get_logger().info(f'Received direction command: {direction_str}')
        
    def twist_callback(self, msg):
        """Handle Twist messages from cmd_vel topic"""
        linear_x = msg.linear.x
        
        # แก้ไข: คำนวณความเร็วและทิศทางจาก linear_x อย่างถูกต้อง
        if linear_x >= 0:
            self.current_direction = True  # Forward
            self.current_speed = min(100.0, linear_x * 100.0)  # แปลงกลับเป็น % โดยไม่ใช้ abs()
        else:
            self.current_direction = False  # Backward
            self.current_speed = min(100.0, abs(linear_x) * 100.0)  # ใช้ abs() เฉพาะกับค่าลบ
            
        self.update_motor()
        self.get_logger().info(f'Twist command - Speed: {self.current_speed}%, Direction: {"Forward" if self.current_direction else "Backward"}')
        
    def update_motor(self):
        """Update H-Bridge motor control based on current settings"""
        try:
            if self.current_speed == 0:
                # Stop motor (brake)
                GPIO.output(self.IN1_PIN, GPIO.LOW)
                GPIO.output(self.IN2_PIN, GPIO.LOW)
                if self.pwm_a:
                    self.pwm_a.ChangeDutyCycle(0)
            else:
                # Set direction and speed
                if self.current_direction:
                    # Forward: IN1=HIGH, IN2=LOW
                    GPIO.output(self.IN1_PIN, GPIO.HIGH)
                    GPIO.output(self.IN2_PIN, GPIO.LOW)
                else:
                    # Backward: IN1=LOW, IN2=HIGH  
                    GPIO.output(self.IN1_PIN, GPIO.LOW)
                    GPIO.output(self.IN2_PIN, GPIO.HIGH)
                
                # Set PWM speed
                if self.pwm_a:
                    self.pwm_a.ChangeDutyCycle(self.current_speed)
            
        except Exception as e:
            self.get_logger().error(f'Failed to update motor: {str(e)}')
            
    def publish_status(self):
        """Publish current motor status"""
        speed_msg = Float32()
        speed_msg.data = self.current_speed
        self.status_publisher.publish(speed_msg)
        
        direction_msg = Bool()
        direction_msg.data = self.current_direction
        self.direction_status_publisher.publish(direction_msg)
        
    def stop_motor(self):
        """Emergency stop function"""
        self.current_speed = 0.0
        self.update_motor()
        self.get_logger().info('Motor stopped')
        
    def cleanup_gpio(self):
        """Clean up GPIO resources"""
        try:
            self.stop_motor()
            if self.pwm_a:
                self.pwm_a.stop()
            if self.pwm_b:
                self.pwm_b.stop()
            GPIO.cleanup()
            self.get_logger().info('GPIO cleaned up')
        except Exception as e:
            self.get_logger().error(f'Error during GPIO cleanup: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    motor_controller = HBridgeMotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info('Shutting down H-Bridge motor controller...')
    finally:
        try:
            motor_controller.cleanup_gpio()
            motor_controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 