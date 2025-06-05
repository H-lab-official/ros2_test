#!/bin/bash

# DC Motor Controller ROS2 Package - Dependency Installation Script
# สำหรับ Raspberry Pi 4

echo "Installing dependencies for DC Motor Controller ROS2 Package..."

# Update system
echo "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install ROS2 (if not already installed)
echo "Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 not found. Please install ROS2 first:"
    echo "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    exit 1
fi

# Install Python GPIO library
echo "Installing RPi.GPIO..."
sudo apt install -y python3-rpi.gpio python3-pip

# Install additional GPIO libraries
echo "Installing additional GPIO libraries..."
pip3 install RPi.GPIO --user

# Install ROS2 development tools
echo "Installing ROS2 development tools..."
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep
sudo apt install -y ros-humble-std-msgs
sudo apt install -y ros-humble-geometry-msgs

# Initialize rosdep (if not already done)
if [ ! -d "/etc/ros/rosdep" ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

rosdep update

# Setup GPIO permissions
echo "Setting up GPIO permissions..."
sudo usermod -a -G gpio $USER
sudo usermod -a -G spi $USER
sudo usermod -a -G i2c $USER

# Create udev rules for GPIO access
echo "Creating GPIO udev rules..."
sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null <<EOF
KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c 'find -L /sys/class/gpio/ -maxdepth 2 -exec chown root:gpio {} \; -exec chmod 664 {} \; || true'"
EOF

sudo udevadm control --reload-rules

echo "Installation completed!"
echo ""
echo "IMPORTANT: You need to logout and login again (or reboot) for GPIO permissions to take effect."
echo ""
echo "To build the package:"
echo "1. Create workspace: mkdir -p ~/ros2_ws/src"
echo "2. Copy this package to: ~/ros2_ws/src/dc_motor_controller"
echo "3. Build: cd ~/ros2_ws && colcon build --packages-select dc_motor_controller"
echo "4. Source: source install/setup.bash"
echo "" 