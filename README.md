# DC Motor Controller ROS2 Package

โปรเจค ROS2 สำหรับควบคุมมอเตอร์ DC ผ่าน PWM บน Raspberry Pi 4

## คุณสมบัติ (Features)

- ควบคุมความเร็วมอเตอร์ DC ผ่าน PWM (0-100%)
- ควบคุมทิศทางการหมุน (หน้า/หลัง)
- รองรับ cmd_vel สำหรับการควบคุมแบบ Twist
- **Keyboard Control 2 แบบ:**
  - **Toggle Mode**: กดเพื่อเปลี่ยนสถานะ
  - **Momentary Mode**: กดค้างเพื่อเคลื่อนที่ (แนะนำ)
- **Realtime Speed Control**: ปรับความเร็วขณะเคลื่อนที่
- Status feedback แบบ real-time
- Emergency stop functionality  
- GPIO cleanup เมื่อปิดโปรแกรม
- รองรับ H-Bridge Motor Driver (Vichagorn)
- Multi-PWM support ผ่าน PCA9685 (8 channels)

## การเชื่อมต่อ Hardware

### GPIO Pin Configuration (BCM numbering)

#### สำหรับ Simple Motor Driver:
- **Physical Pin 32 = GPIO 12**: PWM output สำหรับควบคุมความเร็ว (PWM0)
- **Physical Pin 33 = GPIO 13**: Direction control สำหรับควบคุมทิศทาง (PWM1)
- **Physical Pin 25 = GND**: Ground reference (เชื่อมกับ GND ของมอเตอร์)

#### สำหรับ H-Bridge Motor Driver (Vichagorn):
**Hardware PWM Pins available:** GPIO 12, 13, 18, 19

**Motor A:**
- **Physical Pin 32 = GPIO 12**: ENA (Hardware PWM0)
- **Physical Pin 33 = GPIO 13**: IN1 (Direction control)
- **Physical Pin 35 = GPIO 19**: IN2 (Direction control)

**Motor B (รองรับในอนาคต):**
- **Physical Pin 12 = GPIO 18**: ENB (Hardware PWM1)
- **Physical Pin 37 = GPIO 26**: IN3 (Direction control)
- **Physical Pin 38 = GPIO 20**: IN4 (Direction control)

### วงจร Motor Driver

#### Simple Motor Driver:
```
Raspberry Pi 4           Motor Driver          DC Motor
Pin 32 (GPIO 12/PWM) --> PWM Input
Pin 33 (GPIO 13/DIR) --> Direction Input
Pin 25 (GND) ---------> GND              --> Motor GND
5V/3.3V --------------> VCC
                        Motor OUT+ --------> Motor +
                        Motor OUT- --------> Motor -
```

#### H-Bridge Motor Driver (Vichagorn):
```
Raspberry Pi 4           H-Bridge Driver       Motors
Pin 32 (GPIO 12) -----> ENA (Hardware PWM0) --> Motor A Speed
Pin 33 (GPIO 13) -----> IN1              --> Motor A Dir
Pin 35 (GPIO 19) -----> IN2              --> Motor A Dir
Pin 12 (GPIO 18) -----> ENB (Hardware PWM1) --> Motor B Speed  
Pin 37 (GPIO 26) -----> IN3              --> Motor B Dir
Pin 38 (GPIO 20) -----> IN4              --> Motor B Dir
Pin 25 (GND) ---------> GND              --> Common GND
5V ----------------> +5V (Logic Power)
12V/24V ------------> VCC (Motor Power)
                        OUT1/OUT2 ---------> Motor A (+/-)
                        OUT3/OUT4 ---------> Motor B (+/-)
```

## การติดตั้ง (Installation)

### 1. Prerequisites บน Raspberry Pi 4

```bash
# ติดตั้ง ROS2 (Humble/Iron)
sudo apt update
sudo apt install ros-humble-desktop

# ติดตั้ง Python GPIO library
sudo apt install python3-rpi.gpio

# ติดตั้ง dependencies
sudo apt install python3-pip
pip3 install RPi.GPIO

# แก้ปัญหา GPIO permissions
sudo usermod -a -G gpio $USER
# logout และ login ใหม่หรือ reboot
```

### 2. สร้าง ROS2 Sudo Wrapper (สำคัญ!)

```bash
# สร้าง wrapper script สำหรับรัน ROS2 ด้วย sudo
sudo tee /usr/local/bin/ros2-sudo > /dev/null << 'EOF'
#!/bin/bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
source /opt/ros/humble/setup.bash
source /home/$SUDO_USER/ros2_ws/install/setup.bash 2>/dev/null || true
exec ros2 "$@"
EOF

sudo chmod +x /usr/local/bin/ros2-sudo
```

### 3. สร้าง Workspace และ Build

```bash
# สร้าง workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone โปรเจค
git clone https://github.com/H-lab-official/ros2_test.git dc_motor_controller
# หรือ คัดลอกไฟล์ทั้งหมดจากโปรเจคนี้

# กลับไปที่ workspace root
cd ~/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build package
colcon build --packages-select dc_motor_controller

# Source workspace
source install/setup.bash
```

### 4. สร้าง Executables (หาก Build ไม่สร้างให้)

```bash
# สร้าง directory สำหรับ executables
mkdir -p ~/ros2_ws/install/dc_motor_controller/lib/dc_motor_controller/

# ไปยัง directory
cd ~/ros2_ws/install/dc_motor_controller/lib/dc_motor_controller/

# สร้าง keyboard_control_momentary executable
cat > keyboard_control_momentary << 'EOF'
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/hobot/ros2_ws/install/dc_motor_controller/lib/python3.10/site-packages')
from dc_motor_controller.keyboard_control_momentary import main
if __name__ == '__main__':
    main()
EOF

chmod +x keyboard_control_momentary
```

## การใช้งาน (Usage)

### 1. รัน Motor Controller Node

```bash
# Source environment
source ~/ros2_ws/install/setup.bash

# สำหรับ Simple Motor Driver
sudo /usr/local/bin/ros2-sudo run dc_motor_controller motor_controller_node

# สำหรับ H-Bridge Motor Driver (แนะนำ)
sudo /usr/local/bin/ros2-sudo run dc_motor_controller h_bridge_motor_controller

# หรือใช้ launch file
sudo /usr/local/bin/ros2-sudo launch dc_motor_controller motor_launch.py
```

### 2. รัน Keyboard Control

#### แบบ Momentary (แนะนำ) - กดค้างเพื่อเคลื่อนที่

```bash
# รัน momentary keyboard control พร้อม motor controller  
sudo /usr/local/bin/ros2-sudo launch dc_motor_controller motor_momentary_launch.py

# หรือรันแยกในเทอร์มินัลต่างหาก
sudo /usr/local/bin/ros2-sudo run dc_motor_controller keyboard_control_momentary
```

**การใช้งาน Momentary Keyboard Control:**
- **W/w** : กดค้างเพื่อเดินหน้า (ปล่อยแล้วหยุด)
- **S/s** : กดค้างเพื่อถอยหลัง (ปล่อยแล้วหยุด)
- **+** : เพิ่มความเร็วพื้นฐาน (+10%) - ใช้ได้ขณะเคลื่อนที่
- **-** : ลดความเร็วพื้นฐาน (-10%) - ใช้ได้ขณะเคลื่อนที่
- **0** : รีเซ็ตความเร็วเป็น 30%
- **Q/q** : ออกจากโปรแกรม

**💡 คุณสมบัติพิเศษ:**
- **Realtime Speed Control**: กด +/- ขณะกด W/S เพื่อปรับความเร็วแบบ realtime
- **Consistent Speed**: ทั้ง W และ S ใช้ความเร็วเดียวกัน
- **Low Latency**: Response time เพียง 50ms

#### แบบ Toggle - กดเพื่อเปลี่ยนสถานะ

```bash
# รัน toggle keyboard control พร้อม motor controller
sudo /usr/local/bin/ros2-sudo launch dc_motor_controller motor_keyboard_launch.py

# หรือรันแยกในเทอร์มินัลต่างหาก
sudo /usr/local/bin/ros2-sudo run dc_motor_controller keyboard_control_node
```

**การใช้งาน Toggle Keyboard Control:**
- **W/w** : เดินหน้า (Forward)
- **S/s** : ถอยหลัง (Backward)  
- **+** : เพิ่มความเร็ว (+5%)
- **-** : ลดความเร็ว (-5%)
- **0** : หยุดมอเตอร์ (Stop)
- **Q/q** : ออกจากโปรแกรม (Quit)

### 3. รัน Test Node (สำหรับทดสอบ)

```bash
# รัน test node ในเทอร์มินัลแยก
ros2 run dc_motor_controller motor_test_node

# หรือ launch พร้อมกับ controller
ros2 launch dc_motor_controller motor_launch.py run_test_node:=true
```

### 4. ส่งคำสั่งด้วยมือ (Manual Commands)

#### ส่งคำสั่งความเร็ว (Speed: 0-100%)
```bash
# ความเร็ว 50%
ros2 topic pub /motor/speed std_msgs/msg/Float32 "data: 50.0"

# ความเร็ว 0% (หยุด)
ros2 topic pub /motor/speed std_msgs/msg/Float32 "data: 0.0"
```

#### ส่งคำสั่งทิศทาง (Direction)
```bash
# หมุนไปข้างหน้า (Forward)
ros2 topic pub /motor/direction std_msgs/msg/Bool "data: true"

# หมุนไปข้างหลัง (Backward)  
ros2 topic pub /motor/direction std_msgs/msg/Bool "data: false"
```

#### ส่งคำสั่งด้วย cmd_vel
```bash
# เคลื่อนที่ไปข้างหน้าความเร็ว 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# เคลื่อนที่ไปข้างหลังความเร็ว 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# หยุด
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### 5. ตรวจสอบ Status

```bash
# ตรวจสอบความเร็วปัจจุบัน
ros2 topic echo /motor/status/speed

# ตรวจสอบทิศทางปัจจุบัน
ros2 topic echo /motor/status/direction

# ดู topic ทั้งหมด
ros2 topic list

# ดูโครงสร้าง topic
ros2 topic info /motor/speed
```

## Available Nodes & Launch Files

### Nodes
- `motor_controller_node` - Simple motor controller
- `h_bridge_motor_controller` - H-Bridge motor controller (แนะนำ)
- `keyboard_control_node` - Toggle keyboard control
- `keyboard_control_momentary` - Momentary keyboard control (แนะนำ)
- `motor_test_node` - Test automation
- `multi_pwm_controller` - Multi-channel PWM via PCA9685

### Launch Files
- `motor_launch.py` - รัน motor controller อย่างเดียว
- `motor_keyboard_launch.py` - รัน motor controller + toggle keyboard
- `motor_momentary_launch.py` - รัน motor controller + momentary keyboard (แนะนำ)

## ROS2 Topics

### Subscribed Topics (รับคำสั่ง)
- `/motor/speed` (std_msgs/Float32): ความเร็วมอเตอร์ 0-100%
- `/motor/direction` (std_msgs/Bool): ทิศทาง (true=หน้า, false=หลัง)
- `/cmd_vel` (geometry_msgs/Twist): คำสั่งเคลื่อนที่แบบ Twist

### Published Topics (ส่งสถานะ)
- `/motor/status/speed` (std_msgs/Float32): ความเร็วปัจจุบัน
- `/motor/status/direction` (std_msgs/Bool): ทิศทางปัจจุบัน

## Configuration

แก้ไขค่าต่างๆ ใน `config/motor_config.yaml`:

```yaml
motor_controller:
  ros__parameters:
    pwm_pin: 12              # Physical pin 32 = GPIO 12 (PWM0)
    direction_pin: 13        # Physical pin 33 = GPIO 13 (PWM1)
    pwm_frequency: 1000      # ความถี่ PWM (Hz)
    max_speed: 100.0         # ความเร็วสูงสุด (%)
```

## PWM Information

### Raspberry Pi 4 Hardware PWM
- **Available Pins**: GPIO 12, 13, 18, 19
- **Channels**: 2 independent PWM channels (PWM0, PWM1)
- **Resolution**: 1-bit (On/Off with frequency control)
- **Max Frequency**: ~25 MHz

### Raspberry Pi 5 Hardware PWM  
- **Available Pins**: GPIO 12, 13, 18, 19 (PWM0), GPIO 2, 3, 6, 7 (PWM1)
- **Channels**: 4 independent PWM channels
- **Resolution**: 16-bit (65536 levels)
- **Max Frequency**: ~125 MHz

### การเพิ่ม PWM Channels
1. **Software PWM** - ใช้ GPIO pins ธรรมดา (ความแม่นยำต่ำ)
2. **PCA9685** - I2C PWM controller, 16 channels, 12-bit resolution (แนะนำ)
3. **Arduino as Slave** - ใช้ Arduino เป็น PWM generator

## Troubleshooting

### GPIO Permission Error
```bash
# เพิ่มสิทธิ์ GPIO
sudo usermod -a -G gpio $USER
# logout และ login ใหม่หรือ reboot

# หรือใช้ ros2-sudo wrapper ที่สร้างไว้แล้ว
sudo /usr/local/bin/ros2-sudo run dc_motor_controller motor_controller_node
```

### No Executable Found
```bash
# ตรวจสอบว่า executable ถูกสร้าง
ls -la ~/ros2_ws/install/dc_motor_controller/lib/dc_motor_controller/

# หากไม่มี ให้สร้าง manual (ดูในส่วน Installation step 4)

# หรือ rebuild package
cd ~/ros2_ws
rm -rf build/dc_motor_controller install/dc_motor_controller
colcon build --packages-select dc_motor_controller
```

### Package Not Found
```bash
# ตรวจสอบว่า source workspace แล้ว
source ~/ros2_ws/install/setup.bash

# ตรวจสอบว่า build สำเร็จ
cd ~/ros2_ws
colcon build --packages-select dc_motor_controller
```

### Keyboard Control Issues
```bash
# ตรวจสอบว่า tty support ทำงาน
python3 -c "import tty; print('tty module OK')"

# ตรวจสอบว่า termios ทำงาน  
python3 -c "import termios; print('termios module OK')"

# รัน keyboard control ใน terminal ที่รองรับ tty
```

### Motor ไม่เคลื่อนไหว
1. ตรวจสอบการเชื่อมต่อ GPIO pins
2. ตรวจสอบ power supply ของ motor driver
3. ตรวจสอบ log ด้วย `ros2 topic echo /rosout`
4. ทดสอบ GPIO ด้วย `gpio readall` (หาก package wiringpi ติดตั้งแล้ว)

### การทดสอบ GPIO
```bash
# ติดตั้ง gpio utilities
sudo apt install wiringpi

# ดู GPIO status
gpio readall

# ทดสอบ PWM pin
gpio mode 1 pwm  # GPIO 18 = wiringPi pin 1
gpio pwm 1 512   # 50% duty cycle
```

## Performance Tips

### สำหรับ Realtime Performance
```bash
# ลด CPU governor เป็น performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# เพิ่ม thread priority ใน code
# (อยู่ใน motor controller แล้ว)
```

### สำหรับ High-frequency PWM
```bash
# เพิ่ม GPU memory split สำหรับ hardware PWM
echo "gpu_mem=16" | sudo tee -a /boot/config.txt
sudo reboot
```

## Safety Notes

⚠️ **ข้อควรระวัง**
- ตรวจสอบการเชื่อมต่อวงจรก่อนเปิดเครื่อง
- ใช้ current limit ที่เหมาะสมกับมอเตอร์
- มี emergency stop button พร้อมใช้งาน
- GPIO จะถูก cleanup อัตโนมัติเมื่อปิดโปรแกรม
- **อย่าลืมปล่อย W/S ในโหมด momentary เพื่อหยุดมอเตอร์**
- ใช้ protective gear เมื่อทดสอบมอเตอร์ความเร็วสูง

## Development Notes

### Code Structure
```
dc_motor_controller/
├── dc_motor_controller/
│   ├── motor_controller_node.py      # Simple motor controller
│   ├── h_bridge_motor_controller.py  # H-Bridge controller (แนะนำ)
│   ├── keyboard_control_node.py      # Toggle keyboard control
│   ├── keyboard_control_momentary.py # Momentary keyboard control
│   ├── motor_test_node.py           # Test automation
│   └── multi_pwm_controller.py      # Multi-PWM via PCA9685
├── launch/
│   ├── motor_launch.py
│   ├── motor_keyboard_launch.py
│   └── motor_momentary_launch.py
├── config/
│   └── motor_config.yaml
├── setup.py
├── package.xml
└── README.md
```

### การเพิ่ม Features ใหม่
1. แก้ไข Python files ใน `dc_motor_controller/`
2. อัพเดท `setup.py` หากเพิ่ม entry points
3. Build package: `colcon build --packages-select dc_motor_controller`
4. Test และ commit เข้า Git

## License

MIT License

## Author

H-lab Official  
สร้างสำหรับการควบคุมมอเตอร์ DC บน Raspberry Pi 4 ด้วย ROS2 