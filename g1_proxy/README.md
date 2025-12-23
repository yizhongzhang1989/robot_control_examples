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

### 2. Run the Demo
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