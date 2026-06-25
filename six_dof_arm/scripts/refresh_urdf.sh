#!/bin/bash

XACRO_FILE="urdf/arm.xacro"
OUTPUT="urdf/arm.urdf"

# Generate URDF
ros2 run xacro xacro $XACRO_FILE -o $OUTPUT
echo "URDF generated at $OUTPUT"

# Kill existing processes
pkill -f robot_state_publisher
pkill -f joint_state_publisher

echo "Done — now run your RViz launch file"