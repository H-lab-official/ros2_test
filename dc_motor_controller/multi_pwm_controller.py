#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
import json

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


class MultiPWMController(Node):
    def __init__(self):
        super().__init__('multi_pwm_controller')
        
        # PWM Pins Configuration (เพิ่ม Software PWM)
        self.pwm_pins = {
            0: 12,  # Hardware PWM0 (Motor A ENA)
            1: 18,  # Hardware PWM1 (Motor B ENA) 
            2: 21,  # Software PWM (Extra PWM 1)
            3: 22,  # Software PWM (Extra PWM 2)
            4: 23,  # Software PWM (Extra PWM 3)
            5: 24,  # Software PWM (Extra PWM 4)
            6: 25,  # Software PWM (Extra PWM 5)
            7: 27,  # Software PWM (Extra PWM 6)
        }
        
        self.pwm_frequency = 1000  # 1kHz
        self.pwm_objects = {}
        self.pwm_values = {pin_id: 0.0 for pin_id in self.pwm_pins}
        
        # Initialize GPIO
        self.setup_gpio()
        
        # ROS2 Subscriptions
        self.pwm_single_subscription = self.create_subscription(
            String,
            'pwm/single',
            self.pwm_single_callback,
            10)
        
        self.pwm_multi_subscription = self.create_subscription(
            String,
            'pwm/multi',
            self.pwm_multi_callback,
            10)
        
        # ROS2 Publishers for status
        self.pwm_status_publisher = self.create_publisher(
            String,
            'pwm/status',
            10)
        
        # Status publishing timer
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info('Multi PWM Controller Node Started')
        self.get_logger().info(f'Available PWM channels: {list(self.pwm_pins.keys())}')
        self.get_logger().info(f'Hardware PWM: 0,1 | Software PWM: 2,3,4,5,6,7')
        
    def setup_gpio(self):
        """Initialize all PWM pins"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            for pin_id, gpio_pin in self.pwm_pins.items():
                GPIO.setup(gpio_pin, GPIO.OUT)
                pwm = GPIO.PWM(gpio_pin, self.pwm_frequency)
                pwm.start(0)
                self.pwm_objects[pin_id] = pwm
                self.get_logger().info(f'PWM {pin_id}: GPIO {gpio_pin} initialized')
            
            self.get_logger().info('All PWM pins initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PWM: {str(e)}')
    
    def pwm_single_callback(self, msg):
        """Handle single PWM command: {"pin": 0, "value": 50.0}"""
        try:
            cmd = json.loads(msg.data)
            pin_id = cmd.get('pin')
            value = cmd.get('value', 0.0)
            
            self.set_pwm(pin_id, value)
            self.get_logger().info(f'PWM {pin_id} set to {value}%')
            
        except Exception as e:
            self.get_logger().error(f'PWM single command error: {e}')
    
    def pwm_multi_callback(self, msg):
        """Handle multiple PWM commands: {"0": 50.0, "1": 75.0, "2": 25.0}"""
        try:
            cmd = json.loads(msg.data)
            
            for pin_str, value in cmd.items():
                pin_id = int(pin_str)
                self.set_pwm(pin_id, value)
            
            self.get_logger().info(f'Multi PWM updated: {cmd}')
            
        except Exception as e:
            self.get_logger().error(f'PWM multi command error: {e}')
    
    def set_pwm(self, pin_id, value):
        """Set PWM value for specific pin"""
        if pin_id not in self.pwm_pins:
            self.get_logger().warning(f'Invalid PWM pin ID: {pin_id}')
            return
            
        # Clamp value between 0-100
        value = max(0.0, min(100.0, value))
        
        try:
            self.pwm_objects[pin_id].ChangeDutyCycle(value)
            self.pwm_values[pin_id] = value
            
        except Exception as e:
            self.get_logger().error(f'Failed to set PWM {pin_id}: {e}')
    
    def publish_status(self):
        """Publish current PWM status"""
        status = {
            'pwm_values': self.pwm_values,
            'pin_mapping': self.pwm_pins
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.pwm_status_publisher.publish(msg)
    
    def stop_all_pwm(self):
        """Stop all PWM outputs"""
        for pin_id in self.pwm_pins:
            self.set_pwm(pin_id, 0.0)
        self.get_logger().info('All PWM stopped')
    
    def cleanup_gpio(self):
        """Clean up GPIO resources"""
        try:
            self.stop_all_pwm()
            for pwm in self.pwm_objects.values():
                pwm.stop()
            GPIO.cleanup()
            self.get_logger().info('PWM GPIO cleaned up')
        except Exception as e:
            self.get_logger().error(f'Error during GPIO cleanup: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    pwm_controller = MultiPWMController()
    
    try:
        rclpy.spin(pwm_controller)
    except KeyboardInterrupt:
        pwm_controller.get_logger().info('Shutting down multi PWM controller...')
    finally:
        try:
            pwm_controller.cleanup_gpio()
            pwm_controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 