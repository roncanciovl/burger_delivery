#!/bin/bash
set -e
export DEBIAN_FRONTEND=noninteractive

echo "Updating apt..."
apt update
apt install -y software-properties-common curl

echo "Adding Universe repo..."
add-apt-repository universe -y

echo "Adding ROS 2 GPG key..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Installing ROS 2 Jazzy Desktop..."
apt update
apt install -y ros-jazzy-desktop ros-dev-tools

echo "ROS 2 Jazzy Installation Complete!"
