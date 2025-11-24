#!/bin/bash
set -e

echo "Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "Creating workspace..."
mkdir -p ~/ros2_ws/src

echo "Linking burger_description package..."
# Remove existing link if it exists
rm -rf ~/ros2_ws/src/burger_description
# Create symbolic link from Windows path (mounted in /mnt/c)
ln -s /mnt/c/Users/Usuario/OneDrive/biblioteca/ROS/burger_description ~/ros2_ws/src/

echo "Building workspace..."
cd ~/ros2_ws
colcon build --packages-select burger_description

echo "Build complete!"
echo "To use the package, run: source ~/ros2_ws/install/setup.bash"
