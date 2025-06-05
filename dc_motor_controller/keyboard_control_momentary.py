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
import select


class KeyboardControlMomentary(Node):
    def __init__(self):
        super().__init__('keyboard_control_momentary')
        
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
        
        # ตัวแปรสำหรับเก็บสถานะ
        self.base_speed = 30.0  # ความเร็วพื้นฐาน (ปรับได้ด้วย +/-)
        self.max_speed = 100.0
        self.speed_increment = 10.0  # เพิ่ม/ลดความเร็วทีละ 10%
        self.is_running = True
        
        # สำหรับ momentary control
        self.keys_pressed = set()  # เก็บปุ่มที่กดค้างอยู่
        self.current_direction = True  # True = Forward, False = Backward
        self.is_moving = False
        
        # สำหรับ keyboard input
        self.old_settings = None
        
        self.get_logger().info('Momentary Keyboard Control Node Started')
        self.print_instructions()
        
        # เริ่ม keyboard input thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Timer สำหรับ check key state
        self.control_timer = self.create_timer(0.1, self.update_motor_control)
        
    def print_instructions(self):
        """แสดงคำแนะนำการใช้งาน"""
        print("\n" + "="*60)
        print("    DC Motor Momentary Keyboard Control")
        print("="*60)
        print("Controls:")
        print("  W/w : กดค้างเพื่อเดินหน้า (Hold for Forward)")
        print("  S/s : กดค้างเพื่อถอยหลัง (Hold for Backward)")
        print("  +   : เพิ่มความเร็วพื้นฐาน (+10%)")
        print("  -   : ลดความเร็วพื้นฐาน (-10%)")
        print("  0   : รีเซ็ตความเร็วเป็น 30%")
        print("  Q/q : ออกจากโปรแกรม (Quit)")
        print("-"*60)
        print("⚠️  หมายเหตุ: ต้องกดปุ่ม W หรือ S ค้างไว้ตลอดเวลา")
        print("   ปล่อยปุ่มแล้วมอเตอร์จะหยุดทันที")
        print("-"*60)
        print(f"Current Base Speed: {self.base_speed}%")
        print("="*60)
        print("Press and HOLD W or S to move...")
        
    def setup_keyboard(self):
        """ตั้งค่า keyboard สำหรับรับ input แบบ real-time"""
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
    
    def restore_keyboard(self):
        """คืนค่า keyboard settings เดิม"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def keyboard_input_loop(self):
        """Loop สำหรับรับ keyboard input แบบ continuous"""
        self.setup_keyboard()
        
        try:
            while self.is_running:
                try:
                    # ใช้ select เพื่อตรวจสอบว่ามี input หรือไม่
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        key = sys.stdin.read(1)
                        if key:
                            self.process_key_press(key)
                    else:
                        # ไม่มี input ใหม่ = ปล่อยปุ่มแล้ว
                        self.handle_no_input()
                        
                except Exception as e:
                    self.get_logger().error(f"Keyboard input error: {e}")
                    break
                    
        finally:
            self.restore_keyboard()
    
    def process_key_press(self, key):
        """ประมวลผล key press"""
        key_lower = key.lower()
        
        if key_lower == 'w':
            self.keys_pressed.add('w')
            self.current_direction = True
            self.get_logger().info("Forward key pressed")
            
        elif key_lower == 's':
            self.keys_pressed.add('s')
            self.current_direction = False
            self.get_logger().info("Backward key pressed")
            
        elif key == '+' or key == '=':
            # เพิ่มความเร็วพื้นฐาน
            self.base_speed = min(self.max_speed, self.base_speed + self.speed_increment)
            self.get_logger().info(f"Base speed increased to: {self.base_speed}%")
            self.print_speed_status()
            
        elif key == '-' or key == '_':
            # ลดความเร็วพื้นฐาน
            self.base_speed = max(10.0, self.base_speed - self.speed_increment)
            self.get_logger().info(f"Base speed decreased to: {self.base_speed}%")
            self.print_speed_status()
            
        elif key == '0':
            # รีเซ็ตความเร็วพื้นฐาน
            self.base_speed = 30.0
            self.get_logger().info(f"Base speed reset to: {self.base_speed}%")
            self.print_speed_status()
            
        elif key_lower == 'q':
            # ออกจากโปรแกรม
            self.get_logger().info("Quitting...")
            self.stop_motor()
            self.is_running = False
            rclpy.shutdown()
    
    def handle_no_input(self):
        """จัดการเมื่อไม่มี input (ปล่อยปุ่มแล้ว)"""
        # Clear keys pressed สำหรับ movement keys
        if 'w' in self.keys_pressed or 's' in self.keys_pressed:
            self.keys_pressed.discard('w')
            self.keys_pressed.discard('s')
    
    def update_motor_control(self):
        """อัพเดทการควบคุมมอเตอร์ตาม key state"""
        should_move = 'w' in self.keys_pressed or 's' in self.keys_pressed
        
        if should_move:
            if not self.is_moving:
                self.is_moving = True
                self.get_logger().info(f"Motor started - {('Forward' if self.current_direction else 'Backward')} at {self.base_speed}%")
            
            # ส่งคำสั่งเคลื่อนที่
            self.send_direction_command(self.current_direction)
            self.send_speed_command(self.base_speed)
            self.print_movement_status()
            
        else:
            if self.is_moving:
                self.is_moving = False
                self.get_logger().info("Motor stopped - key released")
                self.stop_motor()
                self.print_stopped_status()
    
    def send_speed_command(self, speed):
        """ส่งคำสั่งความเร็ว"""
        msg = Float32()
        msg.data = speed
        self.speed_publisher.publish(msg)
        
        # ส่งผ่าน cmd_vel ด้วย
        twist_msg = Twist()
        if self.current_direction:
            twist_msg.linear.x = speed / 100.0  # แปลงเป็น 0-1
        else:
            twist_msg.linear.x = -speed / 100.0  # ค่าลบสำหรับถอยหลัง
        self.twist_publisher.publish(twist_msg)
    
    def send_direction_command(self, forward):
        """ส่งคำสั่งทิศทาง"""
        msg = Bool()
        msg.data = forward
        self.direction_publisher.publish(msg)
    
    def stop_motor(self):
        """หยุดมอเตอร์"""
        self.send_speed_command(0.0)
        
    def print_movement_status(self):
        """แสดงสถานะขณะเคลื่อนที่"""
        direction_str = "🔼 FORWARD" if self.current_direction else "🔽 BACKWARD"
        print(f"\r🚗 MOVING {direction_str} | Speed: {self.base_speed:4.0f}% | Base Speed: {self.base_speed:4.0f}%    ", end='', flush=True)
    
    def print_stopped_status(self):
        """แสดงสถานะขณะหยุด"""
        print(f"\r⏹️  STOPPED | Base Speed: {self.base_speed:4.0f}% | Press and HOLD W/S to move...           ", end='', flush=True)
    
    def print_speed_status(self):
        """แสดงสถานะความเร็วพื้นฐาน"""
        print(f"\r⚡ Base Speed: {self.base_speed:4.0f}% | Press and HOLD W/S to move...                ", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        keyboard_control = KeyboardControlMomentary()
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        keyboard_control.get_logger().info('Shutting down keyboard control...')
    finally:
        keyboard_control.stop_motor()
        keyboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 