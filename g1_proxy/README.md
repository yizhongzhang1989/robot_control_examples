# G1 Proxy Package Setup

## 1. Third-party Dependencies
This package uses `RobotControl-G1` and `a2d_sdk` located in `third_party/`.

```bash
# Add RobotControl-G1 submodule with specific branch
git submodule add -f -b "liluo/iros_change" git@github.com:msraeai/RobotControl-G1.git third_party/RobotControl-G1

# If the branch needs to be changed manually:
cd third_party/RobotControl-G1
git fetch origin
git checkout liluo/iros_change
cd ../..
```

The `a2d_sdk` directory should also be placed in `third_party/` (see A2D SDK Environment Setup below).

## 2. Robot Description (URDF)
The `A2D_Omnipicker` folder **must** be placed inside the `urdf/` directory.

```bash
# Copy the A2D_Omnipicker folder to urdf/
cp -r /path/to/A2D_Omnipicker urdf/

# Verify the structure
ls -la urdf/
# Should contain: A2D_Omnipicker/, g1_left_arm.urdf, g1_right_arm.urdf, ...
```

**Expected Structure:**
```
g1_proxy/
  ├── third_party/
  │   ├── RobotControl-G1/
  │   └── a2d_sdk/
  ├── urdf/
  │   ├── A2D_Omnipicker/
  │   ├── g1_left_arm.urdf
  │   ├── g1_right_arm.urdf
  │   └── ...
```

---

## A2D SDK Environment Setup

```bash
# If the Python environment has not been created yet
conda create -n a2d python=3.10
conda activate a2d

# The a2d_sdk directory will be created in third_party/
# If it has already been successfully pulled and dependencies are installed, you can skip this step.
cd third_party
curl -sSL http://10.42.0.101:8849/install.sh | bash
cd ..

# Install dependencies
pip install -r requirements.txt
conda install -n a2d -c conda-forge libstdcxx-ng -y
pip install imageio[ffmpeg]
conda install -n a2d conda-forge::pinocchio -y

# Load environment variables
source third_party/a2d_sdk/env.sh

# Start the robot service
python third_party/a2d_sdk/robot_service.py -s -c third_party/a2d_sdk/conf/hybrid_deploy_depth53.pbtxt
# You must see logs indicating that the camera has also been started
```

---

## Usage

### 1. Build the package
```bash
cd ~/robot_control_examples
colcon build --packages-select g1_proxy
source install/setup.bash
```
### 2. ROS_DOMAIN_ID Requirement

ROS_DOMAIN_ID MUST NOT be 0 for G1.

Example:
  export ROS_DOMAIN_ID=32

This applies to:
- building
- launching
- controller switching
- topic publishing


### 3. Run the Demo
This will launch both the ROS 2 control stack and the G1 interface node.

```bash
# Activate the a2d environment and source env.sh
conda activate a2d
source third_party/a2d_sdk/env.sh

# Set ROS_DOMAIN_ID (must not be 0)
export ROS_DOMAIN_ID=32

# Launch the demo
ros2 launch g1_proxy g1_demo.launch.py
```
This launch:
- starts cartesian_controllers
- starts proxy ros2_control hardware interface
- bridges to A2D SDK

### 4. Controller Switching

Unlike DUCO examples, G1 uses arm-scoped controller managers.

Example (right arm):
```bash
  /right_arm/controller_manager
```
Always specify --controller-manager explicitly.

Default controller:
  cartesian_compliance_controller

List controllers:
```bash
  ros2 control list_controllers \\
    --controller-manager /right_arm/controller_manager
```

Switch compliance → force:
```bash
  ros2 control switch_controllers \\
    --controller-manager /right_arm/controller_manager \\
    --deactivate cartesian_compliance_controller \\
    --activate cartesian_force_controller
```

### 5. Command Topics

Pose command topic:
```bash
  /left_arm/target_frame_left
```
or
```bash
  /right_arm/target_frame_right
```
Type: geometry_msgs/msg/PoseStamped

Example:
```bash
  ros2 topic pub -r 10 /left_arm/target_frame_left geometry_msgs/msg/PoseStamped '
  header:
    frame_id: base_link_l
  pose:
    position:
      x: 0.31300597
      y: -0.03938164
      z: 0.67226203
    orientation:
      x: 0.029901469221476695
      y: 0.6291000972601712
      z: 0.016560979815337943
      w: 0.7765724072571701
'
```

Wrench command topic:
```bash
  /left_arm/target_wrench_left
```
or
```bash
  /right_arm/target_wrench_right
```
  Type: geometry_msgs/msg/WrenchStamped

Frame MUST be end-effector link.

Example:
```bash
  ros2 topic pub -r 10 /left_arm/target_wrench_left geometry_msgs/msg/WrenchStamped '
  header:
    frame_id: Link7_l
  wrench:
    force:
      x: 0.0
      y: 0.0
      z: 0.0
    torque:
      x: 0.0
      y: 0.0
      z: 0.0
'
```
