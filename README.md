# Robot Control Examples

This repository demonstrates cartesian controller integration with different robots using ROS2 control framework. The design separates generic cartesian controllers from robot-specific hardware interfaces, making it easy to add support for new robots.

## Architecture

- **cartesian_controllers_ros2/** - Generic cartesian controllers (motion, force, compliance)
- **duco_ros2_driver/** - DUCO robot-specific hardware interface and MoveIt2 configuration

Both are included as git submodules for modular management.

## System Requirements

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS2**: Humble Hawksbill or later
- **Build System**: colcon

## Installation

### 1. Clone Repository with Submodules

```bash
cd ~/Documents
git clone --recursive https://github.com/yizhongzhang1989/robot_control_examples.git
cd robot_control_examples
```

Or if already cloned, initialize submodules:

```bash
git submodule update --init --recursive
```

### 2. Install System Dependencies

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install -y ros-humble-desktop

# Install required ROS2 packages
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-xacro
```

### 3. Install Package Dependencies

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Install dependencies for all packages
cd ~/Documents/robot_control_examples
rosdep install --from-paths cartesian_controllers_ros2 duco_ros2_driver --ignore-src -y
```

### 4. Build the Workspace

Since this repository contains two submodules with multiple packages, build them together:

```bash
cd ~/Documents/robot_control_examples

# Build all packages (excluding simulation and tests for faster build)
colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or build everything including simulation:
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Build time**: First build may take 5-10 minutes depending on your system.

### 5. Source the Workspace

```bash
source install/setup.bash
```

Add to your `~/.bashrc` for convenience:
```bash
echo "source ~/Documents/robot_control_examples/install/setup.bash" >> ~/.bashrc
```

## Usage

### With DUCO Robot

```bash
# Launch with real robot hardware
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py robot_ip:=192.168.1.10

# Or use fake hardware for testing
ros2 launch duco_gcr5_910_moveit_config demo.launch.py
```

### Cartesian Controllers

The workspace includes three types of cartesian controllers:
- **cartesian_motion_controller** - Tracks Cartesian poses
- **cartesian_force_controller** - Tracks force/torque  
- **cartesian_compliance_controller** - Balances motion and force

See individual package READMEs for detailed usage instructions.

## Adding New Robots

To add support for a new robot:

1. Create a new git submodule with your robot driver
2. Implement `hardware_interface::SystemInterface` for your robot
3. Provide URDF/xacro with ros2_control tags
4. Create controller configuration YAML files
5. Use the existing cartesian controllers without modification

The cartesian controllers work with any robot through standard ROS2 control interfaces.

## Troubleshooting

### Build Errors

If you encounter build errors:

```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Rebuild with verbose output
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
```

### Submodule Issues

```bash
# Update submodules to latest
git submodule update --remote

# Reset submodules if corrupted
git submodule deinit -f .
git submodule update --init --recursive
```

## Repository Structure

```
robot_control_examples/
├── cartesian_controllers_ros2/     (submodule)
│   ├── cartesian_controllers/
│   │   ├── cartesian_motion_controller/
│   │   ├── cartesian_force_controller/
│   │   ├── cartesian_compliance_controller/
│   │   └── cartesian_controller_base/
│   └── cartesian_controller_tools/
└── duco_ros2_driver/              (submodule)
    └── src/
        ├── duco_hardware/
        ├── duco_gcr5_910_moveit_config/
        └── duco_support/
```

## License

See individual submodule licenses:
- cartesian_controllers_ros2: BSD-3-Clause
- duco_ros2_driver: BSD

## References

- [Cartesian Controllers Documentation](cartesian_controllers_ros2/cartesian_controllers/README.md)
- [DUCO Driver Documentation](duco_ros2_driver/README.md)
- [ROS2 Control Framework](https://control.ros.org/)