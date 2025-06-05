# DC Motor Controller ROS2 Package

โปรเจค ROS2 สำหรับควบคุมมอเตอร์ DC ผ่าน PWM บน Raspberry Pi 4

## คุณสมบัติ (Features)

- ควบคุมความเร็วมอเตอร์ DC ผ่าน PWM (0-100%)
- ควบคุมทิศทางการหมุน (หน้า/หลัง)
- รองรับ cmd_vel สำหรับการควบคุมแบบ Twist
- Status feedback แบบ real-time
- Emergency stop functionality
- GPIO cleanup เมื่อปิดโปรแกรม

## การเชื่อมต่อ Hardware

### GPIO Pin Configuration (BCM numbering)
- **Physical Pin 32 = GPIO 12**: PWM output สำหรับควบคุมความเร็ว (PWM0)
- **Physical Pin 33 = GPIO 13**: Direction control สำหรับควบคุมทิศทาง (PWM1)
- **Physical Pin 25 = GND**: Ground reference (เชื่อมกับ GND ของมอเตอร์)

### วงจร Motor Driver
```
Raspberry Pi 4           Motor Driver          DC Motor
Pin 32 (GPIO 12/PWM) --> PWM Input
Pin 33 (GPIO 13/DIR) --> Direction Input
Pin 25 (GND) ---------> GND              --> Motor GND
5V/3.3V --------------> VCC
                        Motor OUT+ --------> Motor +
                        Motor OUT- --------> Motor -
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
```

### 2. สร้าง Workspace และ Build

```bash
# สร้าง workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# คัดลอกโปรเจคนี้ไปยัง src folder
git clone <repository-url> dc_motor_controller
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

## การใช้งาน (Usage)

### 1. รัน Motor Controller Node

```bash
# Source environment
source ~/ros2_ws/install/setup.bash

# รัน motor controller node
ros2 run dc_motor_controller motor_controller_node

# หรือใช้ launch file
ros2 launch dc_motor_controller motor_launch.py
```

### 2. รัน Test Node (สำหรับทดสอบ)

```bash
# รัน test node ในเทอร์มินัลแยก
ros2 run dc_motor_controller motor_test_node

# หรือ launch พร้อมกับ controller
ros2 launch dc_motor_controller motor_launch.py run_test_node:=true
```

### 3. ส่งคำสั่งด้วยมือ (Manual Commands)

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

### 4. ตรวจสอบ Status

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

## Troubleshooting

### GPIO Permission Error
```bash
# เพิ่มสิทธิ์ GPIO
sudo usermod -a -G gpio $USER
# logout และ login ใหม่
```

### Package Not Found
```bash
# ตรวจสอบว่า source workspace แล้ว
source ~/ros2_ws/install/setup.bash

# ตรวจสอบว่า build สำเร็จ
cd ~/ros2_ws
colcon build --packages-select dc_motor_controller
```

### Motor ไม่เคลื่อนไหว
1. ตรวจสอบการเชื่อมต่อ GPIO pins
2. ตรวจสอบ power supply ของ motor driver
3. ตรวจสอบ log ด้วย `ros2 topic echo /rosout`

## Safety Notes

⚠️ **ข้อควรระวัง**
- ตรวจสอบการเชื่อมต่อวงจรก่อนเปิดเครื่อง
- ใช้ current limit ที่เหมาะสมกับมอเตอร์
- มี emergency stop button พร้อมใช้งาน
- GPIO จะถูก cleanup อัตโนมัติเมื่อปิดโปรแกรม

## License

MIT License 