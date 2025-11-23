# Burger Description Package

This package contains the URDF description for the Burger Delivery Robot.

## Prerequisites

- ROS 2 Jazzy
- `colcon` build tool

## Installation

1.  Clone or copy this package into your ROS 2 workspace `src` directory.
    ```bash
    mkdir -p ~/ros2_ws/src
    cp -r /path/to/burger_description ~/ros2_ws/src/
    ```

2.  Build the workspace.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select burger_description
    ```

3.  Source the setup script.
    ```bash
    source install/setup.bash
    ```

## Usage

To visualize the robot model in RViz:

```bash
ros2 launch burger_description display.launch.py
```

This will launch:
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
