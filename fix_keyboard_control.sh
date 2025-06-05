#!/bin/bash

# Fix script for keyboard control node
echo "Fixing keyboard control node..."

# Path to the source file
SOURCE_FILE="dc_motor_controller/keyboard_control_node.py"

# Check if source file exists
if [ ! -f "$SOURCE_FILE" ]; then
    echo "Error: Source file not found: $SOURCE_FILE"
    exit 1
fi

# Apply the fix
sed -i 's/tty\.cbreak(/tty.setcbreak(/g' "$SOURCE_FILE"

echo "Applied fix: tty.cbreak() -> tty.setcbreak()"

# Rebuild the package
echo "Rebuilding package..."
colcon build --packages-select dc_motor_controller

echo "Fix applied and package rebuilt!"
echo ""
echo "Now you can test with:"
echo "sudo /usr/local/bin/ros2-sudo run dc_motor_controller keyboard_control_node" 