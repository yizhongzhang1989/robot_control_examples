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

#### Using Cartesian Controllers (Recommended)

```bash
# Launch with real robot hardware and cartesian controllers
ros2 launch duco_gcr5_910_moveit_config cartesian_controller.launch.py robot_ip:=192.168.1.10

# The compliance controller is active by default for safe operation
# Switch between controllers using:
ros2 control switch_controllers --deactivate cartesian_compliance_controller --activate cartesian_motion_controller
ros2 control switch_controllers --deactivate cartesian_motion_controller --activate cartesian_force_controller
```

#### Using MoveIt2 with Joint Trajectory Controller

```bash
# Launch with real robot hardware and MoveIt2
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py robot_ip:=192.168.1.10

# Or use fake hardware for testing without physical robot
ros2 launch duco_gcr5_910_moveit_config demo.launch.py
```

### Cartesian Controllers

The workspace includes three types of cartesian controllers:
- **cartesian_motion_controller** - Tracks Cartesian poses in real-time (teleoperation, teaching)
- **cartesian_force_controller** - Tracks desired forces/torques (contact tasks)
- **cartesian_compliance_controller** - Balances motion and force (compliant assembly)

#### Controller Switching

Only one Cartesian controller can be active at a time. Switch controllers using:

```bash
# List available controllers
ros2 control list_controllers

# Switch to motion controller (for free-space motion)
ros2 control switch_controllers \
    --deactivate cartesian_compliance_controller cartesian_force_controller \
    --activate cartesian_motion_controller

# Switch to force controller (for contact tasks)
ros2 control switch_controllers \
    --deactivate cartesian_compliance_controller cartesian_motion_controller \
    --activate cartesian_force_controller

# Switch to compliance controller (for compliant motion with force feedback)
ros2 control switch_controllers \
    --deactivate cartesian_motion_controller cartesian_force_controller \
    --activate cartesian_compliance_controller
```

See individual package READMEs for detailed usage instructions and parameter tuning.

## Adding New Robots

To add support for a new robot:

1. **Create robot driver submodule**
   ```bash
   git submodule add <your-robot-repo-url> <robot_name>_driver
   ```

2. **Implement hardware interface**
   - Create a class inheriting from `hardware_interface::SystemInterface`
   - Implement `read()` and `write()` methods for your robot's communication protocol
   - Export as a plugin in your package

3. **Create robot configuration**
   - URDF/xacro with ros2_control tags
   - Joint limits and kinematics configuration
   - Initial positions

4. **Create Cartesian controller configuration**
   - Copy `cartesian_controller_manager.yaml` from DUCO example
   - Adjust joint names, link names, and PD gains for your robot
   - Place in `<robot_name>_moveit_config/config/`

5. **Create launch file**
   - Copy `cartesian_controller.launch.py` from DUCO example
   - Update robot-specific parameters (IP, port, etc.)
   - Place in `<robot_name>_moveit_config/launch/`

6. **Build and test**
   ```bash
   colcon build --packages-select <your-robot-packages>
   source install/setup.bash
   ros2 launch <robot_name>_moveit_config cartesian_controller.launch.py
   ```

The cartesian controllers work with any robot through standard ROS2 control interfaces - no modification to the controller code is needed!

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
        │   ├── config/
        │   │   ├── cartesian_controller_manager.yaml  # Cartesian controller config for DUCO
        │   │   └── ros2_controllers.yaml              # Joint trajectory controller config
        │   └── launch/
        │       ├── cartesian_controller.launch.py     # Launch with Cartesian controllers
        │       └── demo_ros2_control.launch.py        # Launch with MoveIt2
        └── duco_support/
```

### File Organization Philosophy

**Cartesian controller launch files and configurations belong in the robot-specific package** (e.g., `duco_gcr5_910_moveit_config`) because:

1. **Robot-Specific Integration**: The launch file combines robot hardware interface with cartesian controllers
2. **Configuration Dependencies**: Controller parameters (joint names, link names, gains) are robot-specific
3. **Multiple Control Options**: Each robot package can offer different launch files:
   - `cartesian_controller.launch.py` - For Cartesian control (motion/force/compliance)
   - `demo_ros2_control.launch.py` - For MoveIt2 planning with joint trajectory control
4. **Reusability**: The cartesian controller packages remain generic and unchanged across robots

## License

See individual submodule licenses:
- cartesian_controllers_ros2: BSD-3-Clause
- duco_ros2_driver: BSD

## Design Decisions

### Why are Cartesian controller launch files in the robot package?

The `cartesian_controller.launch.py` and `cartesian_controller_manager.yaml` files are placed in `duco_gcr5_910_moveit_config` (robot-specific package) rather than in the generic `cartesian_controllers_ros2` package because:

1. **Integration Layer**: These files integrate robot-specific hardware with generic controllers
2. **Robot-Specific Parameters**: 
   - Joint names: `arm_1_joint_1` through `arm_1_joint_6` (DUCO-specific)
   - Link names: `link_6`, `base_link` (DUCO URDF-specific)
   - PD gains: Tuned specifically for DUCO robot dynamics
3. **Hardware Interface Connection**: Launch file connects to DUCO hardware via IP/port parameters
4. **Controller Options**: Allows each robot package to provide multiple control options:
   - Cartesian control (this file)
   - Joint trajectory control with MoveIt2 (other launch files)
5. **Separation of Concerns**: 
   - `cartesian_controllers_ros2/` = Generic, robot-agnostic controller algorithms
   - `duco_ros2_driver/` = DUCO-specific hardware interface + integration configurations

This design pattern makes it easy to add new robots without modifying the generic controller packages.

## References

- [Cartesian Controllers Documentation](cartesian_controllers_ros2/cartesian_controllers/README.md)
- [DUCO Driver Documentation](duco_ros2_driver/README.md)
- [ROS2 Control Framework](https://control.ros.org/)