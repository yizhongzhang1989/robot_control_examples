# UR Cartesian Control

This package provides launch files and configuration for controlling Universal Robots with cartesian controllers.

## Package Contents

- **Launch Files**: Located in `launch/`
  - `ur_control.launch.py`: Main launch file for bringing up UR robot with cartesian controllers

- **Config Files**: Located in `config/`
  - `ur_controllers.yaml`: Controller configuration including cartesian compliance, force, and motion controllers

## Usage

To launch the UR robot with cartesian control:

```bash
ros2 launch ur_cartesian_control ur_control.launch.py ur_type:=<robot_type> robot_ip:=<robot_ip>
```

Replace `<robot_type>` with your robot model (e.g., ur5e, ur10e) and `<robot_ip>` with your robot's IP address.

### Example

```bash
ros2 launch ur_cartesian_control ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.100
```

## Available Controllers

This configuration includes the following cartesian controllers:

- **cartesian_compliance_controller**: Compliant motion in cartesian space
- **cartesian_force_controller**: Force control in cartesian space  
- **cartesian_motion_controller**: Motion control in cartesian space

Plus standard UR controllers:
- joint_trajectory_controller
- scaled_joint_trajectory_controller
- forward_velocity_controller
- forward_position_controller
- And more...

## Dependencies

- ur_robot_driver
- ur_description
- ur_controllers
- cartesian_compliance_controller
- cartesian_force_controller
- cartesian_motion_controller

Note: The UR packages should be installed via apt (`ros-humble-ur*`) or built from source.
