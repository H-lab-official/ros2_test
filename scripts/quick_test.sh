#!/bin/bash

# Quick Test Script for DC Motor Controller
# ใช้สำหรับทดสอบว่า motor controller ทำงานได้หรือไม่

echo "DC Motor Controller - Quick Test"
echo "=================================="

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 not found. Please source ROS2 environment:"
    echo "source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if package is built
if ! ros2 pkg list | grep -q "dc_motor_controller"; then
    echo "Error: dc_motor_controller package not found."
    echo "Please build the package first:"
    echo "cd ~/ros2_ws && colcon build --packages-select dc_motor_controller"
    echo "source install/setup.bash"
    exit 1
fi

echo "Package found. Starting tests..."
echo ""

# Function to send command and wait
send_command() {
    echo "Sending: $1"
    eval $1
    sleep 2
}

echo "Starting motor controller in background..."
ros2 run dc_motor_controller motor_controller_node &
CONTROLLER_PID=$!

# Wait for controller to start
sleep 3

echo ""
echo "Running test sequence..."
echo "========================"

# Test 1: Set speed to 30%
send_command "ros2 topic pub /motor/speed std_msgs/msg/Float32 \"data: 30.0\" --once"

# Test 2: Set direction forward
send_command "ros2 topic pub /motor/direction std_msgs/msg/Bool \"data: true\" --once"

# Test 3: Increase speed to 60%
send_command "ros2 topic pub /motor/speed std_msgs/msg/Float32 \"data: 60.0\" --once"

# Test 4: Change direction to backward
send_command "ros2 topic pub /motor/direction std_msgs/msg/Bool \"data: false\" --once"

# Test 5: Test with cmd_vel forward
send_command "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"linear: {x: 0.5, y: 0.0, z: 0.0}\" --once"

# Test 6: Test with cmd_vel backward
send_command "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"linear: {x: -0.3, y: 0.0, z: 0.0}\" --once"

# Test 7: Stop motor
send_command "ros2 topic pub /motor/speed std_msgs/msg/Float32 \"data: 0.0\" --once"

echo ""
echo "Test completed. Checking motor status..."

# Check current status
echo "Current motor speed:"
ros2 topic echo /motor/status/speed --once

echo "Current motor direction:"
ros2 topic echo /motor/status/direction --once

echo ""
echo "Stopping motor controller..."
kill $CONTROLLER_PID

echo "Quick test finished!"
echo ""
echo "If you saw no errors and the motor responded to commands, the system is working correctly."
echo "Check the GPIO connections if the motor didn't move." 