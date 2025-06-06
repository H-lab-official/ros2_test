#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import threading
import sys
import termios
import tty
import time


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # Publishers สำหรับส่งคำสั่งไปยัง motor controller
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
        
        # Subscribers สำหรับรับสถานะจาก motor controller
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
        
        # ตัวแปรสำหรับเก็บสถานะ
        self.current_speed = 0.0
        self.current_direction = True  # True = Forward, False = Backward
        self.max_speed = 100.0
        self.speed_increment = 5.0  # เพิ่ม/ลดความเร็วทีละ 5%
        self.is_running = True
        
        # สำหรับ keyboard input
        self.old_settings = None
        
        self.get_logger().info('Keyboard Control Node Started')
        self.print_instructions()
        
        # เริ่ม keyboard input thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
    def print_instructions(self):
        """แสดงคำแนะนำการใช้งาน"""
        print("\n" + "="*50)
        print("    DC Motor Keyboard Control")
        print("="*50)
        print("Controls:")
        print("  W/w : เดินหน้า (Forward)")
        print("  S/s : ถอยหลัง (Backward)")
        print("  +   : เพิ่มความเร็ว (+5%)")
        print("  -   : ลดความเร็ว (-5%)")
        print("  0   : หยุดมอเตอร์ (Stop)")
        print("  Q/q : ออกจากโปรแกรม (Quit)")
        print("-"*50)
        print(f"Current: Speed={self.current_speed}%, Direction={'Forward' if self.current_direction else 'Backward'}")
        print("="*50)
        print("Press keys to control motor...")
        
    def setup_keyboard(self):
        """ตั้งค่า keyboard สำหรับรับ input แบบ real-time"""
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
    
    def restore_keyboard(self):
        """คืนค่า keyboard settings เดิม"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """รับ key input แบบ non-blocking"""
        if sys.stdin.isatty():
            return sys.stdin.read(1)
        return None
    
    def keyboard_input_loop(self):
        """Loop สำหรับรับ keyboard input"""
        self.setup_keyboard()
        
        try:
            while self.is_running:
                try:
                    key = self.get_key()
                    if key:
                        self.process_key(key)
                        time.sleep(0.1)  # หน่วงเวลาเล็กน้อย
                except Exception as e:
                    self.get_logger().error(f"Keyboard input error: {e}")
                    break
        finally:
            self.restore_keyboard()
    
    def process_key(self, key):
        """ประมวลผล key input"""
        key_lower = key.lower()
        
        if key_lower == 'w':
            # เดินหน้า
            self.current_direction = True
            self.send_direction_command(True)
            self.get_logger().info("Direction: Forward")
            
        elif key_lower == 's':
            # ถอยหลัง
            self.current_direction = False
            self.send_direction_command(False)
            self.get_logger().info("Direction: Backward")
            
        elif key == '+' or key == '=':
            # เพิ่มความเร็ว
            self.current_speed = min(self.max_speed, self.current_speed + self.speed_increment)
            self.send_speed_command(self.current_speed)
            self.get_logger().info(f"Speed increased to: {self.current_speed}%")
            
        elif key == '-' or key == '_':
            # ลดความเร็ว
            self.current_speed = max(0.0, self.current_speed - self.speed_increment)
            self.send_speed_command(self.current_speed)
            self.get_logger().info(f"Speed decreased to: {self.current_speed}%")
            
        elif key == '0':
            # หยุดมอเตอร์
            self.current_speed = 0.0
            self.send_speed_command(0.0)
            self.get_logger().info("Motor stopped")
            
        elif key_lower == 'q':
            # ออกจากโปรแกรม
            self.get_logger().info("Quitting...")
            self.current_speed = 0.0
            self.send_speed_command(0.0)
            self.is_running = False
            rclpy.shutdown()
            
        # แสดงสถานะปัจจุบัน
        self.print_status()
    
    def send_speed_command(self, speed):
        """ส่งคำสั่งความเร็ว"""
        msg = Float32()
        msg.data = speed
        self.speed_publisher.publish(msg)
        
        # ส่งผ่าน cmd_vel ด้วย - แก้ไขการคำนวณ
        twist_msg = Twist()
        if self.current_direction:
            twist_msg.linear.x = speed / 100.0  # Forward: บวก
        else:
            twist_msg.linear.x = -(speed / 100.0)  # Backward: ลบ
        self.twist_publisher.publish(twist_msg)
    
    def send_direction_command(self, forward):
        """ส่งคำสั่งทิศทาง"""
        msg = Bool()
        msg.data = forward
        self.direction_publisher.publish(msg)
    
    def print_status(self):
        """แสดงสถานะปัจจุบัน"""
        direction_str = "Forward" if self.current_direction else "Backward"
        print(f"\rSpeed: {self.current_speed:5.1f}% | Direction: {direction_str:8} | Press keys to control...", end='', flush=True)
    
    def speed_status_callback(self, msg):
        """รับสถานะความเร็วจาก motor controller"""
        pass  # อาจใช้สำหรับ sync สถานะในอนาคต
        
    def direction_status_callback(self, msg):
        """รับสถานะทิศทางจาก motor controller"""
        pass  # อาจใช้สำหรับ sync สถานะในอนาคต


def main(args=None):
    rclpy.init(args=args)
    
    try:
        keyboard_control = KeyboardControlNode()
        
        # สร้าง executor สำหรับ non-blocking spin
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(keyboard_control)
        
        # รัน executor ใน thread แยก
        executor_thread = threading.Thread(target=executor.spin)
        executor_thread.daemon = True
        executor_thread.start()
        
        # รอให้ keyboard thread จบ
        keyboard_control.keyboard_thread.join()
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            keyboard_control.restore_keyboard()
            keyboard_control.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 