@echo off
echo DC Motor Controller ROS2 Package - Windows Setup Info
echo ========================================================
echo.
echo This package is designed to run on Raspberry Pi 4 with ROS2.
echo The following commands should be run on your Raspberry Pi:
echo.
echo 1. Install ROS2 Humble:
echo    sudo apt update
echo    sudo apt install ros-humble-desktop
echo.
echo 2. Install GPIO library:
echo    sudo apt install python3-rpi.gpio
echo    pip3 install RPi.GPIO --user
echo.
echo 3. Install development tools:
echo    sudo apt install python3-colcon-common-extensions
echo    sudo apt install python3-rosdep
echo.
echo 4. Setup GPIO permissions:
echo    sudo usermod -a -G gpio $USER
echo.
echo 5. Create workspace and build:
echo    mkdir -p ~/ros2_ws/src
echo    cd ~/ros2_ws/src
echo    [Copy this project to dc_motor_controller folder]
echo    cd ~/ros2_ws
echo    source /opt/ros/humble/setup.bash
echo    colcon build --packages-select dc_motor_controller
echo    source install/setup.bash
echo.
echo 6. Run the motor controller:
echo    ros2 run dc_motor_controller motor_controller_node
echo.
echo Note: This package uses GPIO pins and must run on Raspberry Pi with proper hardware connections.
pause 