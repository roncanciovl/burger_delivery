#!/bin/bash
set -e

echo "Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "Creating workspace..."
mkdir -p ~/ros2_ws/src

echo "Ensuring burger_description is available..."
# In the Linux version, burger_description is already inside burger_delivery/
# We can link it to src/ for colcon if needed, or just let colcon find it.
# If colcon is run from ~/ros2_ws, it will find it inside src/burger_delivery/

echo "Building workspace..."
cd ~/ros2_ws
colcon build --packages-select burger_description

echo "Build complete!"
echo "To use the package, run: source ~/ros2_ws/install/setup.bash"
