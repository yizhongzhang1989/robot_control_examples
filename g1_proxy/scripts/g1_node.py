#!/usr/bin/env python3

# Heecheol Kim | heecheolkim@microsoft.com | MSRA Tokyo | 2025-12-23

import os
import sys
from ament_index_python.packages import get_package_share_directory

# Setup paths for third_party libraries
try:
    pkg_share = get_package_share_directory('g1_proxy')
    third_party_dir = os.path.join(pkg_share, 'third_party')
except Exception:
    # Fallback for running from source
    third_party_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'third_party')

# Add paths
sys.path.append(third_party_dir)
sys.path.append(os.path.join(third_party_dir, 'RobotControl-G1'))
sys.path.append(os.path.join(third_party_dir, 'RobotControl-G1', 'control', 'scripts'))
sys.path.append(os.getcwd())

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../cartesian_controllers_ros2/cartesian_controller_tools'))

from a2d_sdk.robot import RobotDds as Robot

import rclpy
import time
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from ruckig import Ruckig, InputParameter, OutputParameter
from cartesian_controller_tools.ft_compensator import FTCompensator

from a2d_arm_ik import A2D_Arm_FK


# Maximum per-step joint position change (radians) - safety limit for cartesian_controller outputs
MAX_DELTA_POS = 0.02

class G1ArmController:
    def __init__(self, robot, arm_side="left"):
        self.robot = robot
        arm_side = arm_side[0]  # Extract 'l' or 'r'
        self.arm_side = arm_side
        self.otg = Ruckig(7, 0.01)
        self.inp = InputParameter(7)
        self.out = OutputParameter(7)
        self.initialized = False
        self.current_left = []
        self.current_right = []
        self.current_joints = []
        self.arm_poses = None
        self.joint_names = [f'{arm_side}_arm_joint{i}' for i in range(1, 8)]
        
        # Initialize FK Solver with arm_side parameter
        self.fk_solver = G1FKSolver(arm_side=arm_side)
        self.set_limits(2.0, 5.0, 10.0)

    def set_limits(self, vel, acc, jerk):
        self.inp.max_velocity = [vel] * 7
        self.inp.max_acceleration = [acc] * 7
        self.inp.max_jerk = [jerk] * 7

    def update_state(self, arm_joints=None, waist_joints=None):
        if arm_joints is None or waist_joints is None:
            arm_joints, _ = self.robot.arm_joint_states()
            waist_joints, _ = self.robot.waist_joint_states()
            
        if arm_joints is None or len(arm_joints) < 14 or waist_joints is None:
            return False
            
        self.current_left = list(arm_joints[:7])
        self.current_right = list(arm_joints[7:])
        self.current_joints = self.current_left if self.arm_side == 'l' else self.current_right
        
        # Get arm poses (quaternion in xyzw format)
        if self.fk_solver:
            l7_pos, l7_quat_wxyz = self.fk_solver.get_link7_pose(waist_joints, arm_joints)
            # Convert wxyz to xyzw
            self.arm_poses = np.concatenate([l7_pos, np.array([l7_quat_wxyz[1], l7_quat_wxyz[2], l7_quat_wxyz[3], l7_quat_wxyz[0]])])
        
        if not self.initialized:
            curr_joints = self.current_joints
            self.inp.current_position = curr_joints
            self.inp.current_velocity, self.inp.current_acceleration = [0.0]*7, [0.0]*7
            self.inp.target_position = curr_joints
            self.initialized = True
        return True

    def get_next_step(self, target_joints):
        if not self.initialized: return self.current_joints
        self.inp.target_position = target_joints
        self.otg.update(self.inp, self.out)
        self.inp.current_position = self.out.new_position
        self.inp.current_velocity = self.out.new_velocity
        self.inp.current_acceleration = self.out.new_acceleration
        
        # Safety clamp
        safe_position = [
            max(min(self.out.new_position[i], self.current_joints[i] + MAX_DELTA_POS),
                self.current_joints[i] - MAX_DELTA_POS)
            for i in range(7)
        ]
        return safe_position

    def move(self, target_joints):
        new_pos = self.get_next_step(target_joints)
        
        if self.arm_side == 'l':
            self.robot.move_arm(new_pos + self.current_right)
        else:
            self.robot.move_arm(self.current_left + new_pos)

    def get_hand_force(self):
        forces = self.robot.hand_force_states()
        if forces is None: return None
        if len(forces) >= 12:
            return forces[:6] if self.arm_side == 'l' else forces[6:12]
        return forces

class G1FKSolver:
    def __init__(self, arm_side="l"):
        # Use ament_index to find the URDF
        try:
            pkg_share = get_package_share_directory('g1_proxy')
            urdf_dir = os.path.join(pkg_share, 'urdf', 'A2D_Omnipicker')
        except Exception:
            # Fallback for running from source
            script_dir = os.path.dirname(os.path.abspath(__file__))
            urdf_dir = os.path.join(script_dir, '..', 'urdf', 'A2D_Omnipicker')

        urdf_path = os.path.join(urdf_dir, 'A2D.urdf')
        package_dirs = [urdf_dir]
        
        print(f"Loading URDF from: {urdf_path}")
        self.fk_solver = A2D_Arm_FK(urdf_path, package_dirs, arm_side=arm_side)
        self.model = self.fk_solver.reduced_robot.model
        self.data = self.model.createData()
        self.base_idx = self.model.getFrameId(f"base_link_{arm_side}")
        self.link7_idx = self.model.getFrameId(f"Link7_{arm_side}")

    def get_link7_pose(self, waist_joints, arm_joints):
        """
        Returns (pos, quat_wxyz) of Link7_l/r relative to base_link_l/r
        waist_joints: [waist1, waist2]
        arm_joints: [joint1, ..., joint7]
        """
        q_fk = np.array(list(waist_joints[::-1]) + list(arm_joints[:7]), dtype=np.float64)
        
        pin.framesForwardKinematics(self.model, self.data, q_fk)
        T_base = self.data.oMf[self.base_idx].np
        T_link7 = self.data.oMf[self.link7_idx].np
        
        pos = T_link7[:3, 3]
        rot = Rotation.from_matrix(T_link7[:3, :3])
        quat_xyzw = rot.as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        
        return pos, quat_wxyz


class G1Node(Node):
    def __init__(self, robot, force_threshold=5.0, torque_threshold=1.0):
        super().__init__('g1_node')
        self.robot = robot
        self.force_threshold = force_threshold
        self.torque_threshold = torque_threshold
        
        self.initialized = False
        
        # Initialize controllers for both arms
        self.arm_l = G1ArmController(robot, 'left')
        self.arm_r = G1ArmController(robot, 'right')
        
        # Store state for both arms
        self.cmd_position_l = []
        self.cmd_position_r = []
        
        # Store gripper commands
        self.cmd_gripper_l = 0.0
        self.cmd_gripper_r = 0.0
        
        qos = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.RELIABLE)
        
        # Create topics for both arms
        self.pub_state_l = self.create_publisher(JointState, '/robot/left_arm/state', qos)
        self.pub_state_r = self.create_publisher(JointState, '/robot/right_arm/state', qos)
        self.pub_ft_l = self.create_publisher(WrenchStamped, '/robot/left_arm/ft', qos)
        self.pub_ft_r = self.create_publisher(WrenchStamped, '/robot/right_arm/ft', qos)
        
        self.sub_cmd_l = self.create_subscription(
            JointState, '/robot/left_arm/command', self.cmd_callback_l, qos)
        self.sub_cmd_r = self.create_subscription(
            JointState, '/robot/right_arm/command', self.cmd_callback_r, qos)
        
        # Gripper command subscription:
        # cartesian_controller does not publish gripper commands, so
        # external topic publishers must handle gripper commands separately
        self.sub_gripper_l = self.create_subscription(
            JointState, '/robot/left_gripper/command', self.gripper_callback_l, qos)
        self.sub_gripper_r = self.create_subscription(
            JointState, '/robot/right_gripper/command', self.gripper_callback_r, qos)
        
        # Joint names
        self.joint_names_l = self.arm_l.joint_names
        self.joint_names_r = self.arm_r.joint_names
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # Load FT calibration (both arms)
        self.ft_comp_l = FTCompensator('config/ft_calibration_result_l.json')
        self.ft_comp_r = FTCompensator('config/ft_calibration_result_r.json')
        # Sensor frame and end-effector frame differ. This rotation matrix transforms
        # sensor measurements to end-effector frame
        self.R_sensor_ee = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        
        self.get_logger().info("G1 Node initialized for both arms. Waiting...")

    def cmd_callback_l(self, msg):
        if not self.initialized:
            return
        for i, name in enumerate(msg.name):
            if name in self.joint_names_l:
                idx = self.joint_names_l.index(name)
                if i < len(msg.position):
                    self.cmd_position_l[idx] = msg.position[i]

    def cmd_callback_r(self, msg):
        if not self.initialized:
            return
        for i, name in enumerate(msg.name):
            if name in self.joint_names_r:
                idx = self.joint_names_r.index(name)
                if i < len(msg.position):
                    self.cmd_position_r[idx] = msg.position[i]

    def gripper_callback_l(self, msg):
        """Handle left gripper command. Expects position[0] in range [0, 1]"""
        if len(msg.position) > 0:
            self.cmd_gripper_l = np.clip(msg.position[0], 0.0, 1.0)

    def gripper_callback_r(self, msg):
        """Handle right gripper command. Expects position[0] in range [0, 1]"""
        if len(msg.position) > 0:
            self.cmd_gripper_r = np.clip(msg.position[0], 0.0, 1.0)

    def timer_callback(self):
        arm_joints, _ = self.robot.arm_joint_states()
        waist_joints, _ = self.robot.waist_joint_states()
        
        # Update state for both controllers
        if arm_joints is not None and len(arm_joints) >= 14 and waist_joints is not None:
            self.arm_l.update_state(arm_joints, waist_joints)
            self.arm_r.update_state(arm_joints, waist_joints)
        else:
            return

        if not self.initialized:
            if self.arm_l.initialized and self.arm_r.initialized:
                self.get_logger().info("Robot connected. Initializing both arms...")
                self.cmd_position_l = list(self.arm_l.current_joints)
                self.cmd_position_r = list(self.arm_r.current_joints)
                self.initialized = True
            return
        
        # Calculate next step for both arms
        safe_position_l = self.arm_l.get_next_step(self.cmd_position_l)
        safe_position_r = self.arm_r.get_next_step(self.cmd_position_r)
        
        # Move both arms
        full_arm_position = safe_position_l + safe_position_r
        self.robot.move_arm(full_arm_position)
        
        # Move grippers
        gripper_pos = [self.cmd_gripper_l, self.cmd_gripper_r]
        self.robot.move_gripper(gripper_pos)
        
        # Publish left arm state
        msg_l = JointState()
        msg_l.header.stamp = self.get_clock().now().to_msg()
        msg_l.name = self.joint_names_l
        msg_l.position = self.arm_l.current_joints
        self.pub_state_l.publish(msg_l)
        
        # Publish right arm state
        msg_r = JointState()
        msg_r.header.stamp = self.get_clock().now().to_msg()
        msg_r.name = self.joint_names_r
        msg_r.position = self.arm_r.current_joints
        self.pub_state_r.publish(msg_r)
        
        # Publish F/T for both arms
        hand_force = self.robot.hand_force_states()
        if hand_force and len(hand_force) >= 12:
            # Left arm F/T
            if self.arm_l.arm_poses is not None:
                q_xyzw_l = self.arm_l.arm_poses[3:]
                f_raw_l = self.R_sensor_ee @ hand_force[:3]
                t_raw_l = self.R_sensor_ee @ hand_force[3:6]
                
                f_comp_l, t_comp_l = self.ft_comp_l.compensate(f_raw_l, t_raw_l, q_xyzw_l)
                
                # Apply threshold filtering after compensation
                f_comp_l = np.where(np.abs(f_comp_l) < self.force_threshold, 0, f_comp_l)
                t_comp_l = np.where(np.abs(t_comp_l) < self.torque_threshold, 0, t_comp_l)
                
                ft_msg_l = WrenchStamped()
                ft_msg_l.header.stamp = self.get_clock().now().to_msg()
                ft_msg_l.header.frame_id = "Link7_l"
                ft_msg_l.wrench.force.x = f_comp_l[0]
                ft_msg_l.wrench.force.y = f_comp_l[1]
                ft_msg_l.wrench.force.z = f_comp_l[2]
                ft_msg_l.wrench.torque.x = t_comp_l[0]
                ft_msg_l.wrench.torque.y = t_comp_l[1]
                ft_msg_l.wrench.torque.z = t_comp_l[2]
                self.pub_ft_l.publish(ft_msg_l)
            
            # Right arm F/T
            if self.arm_r.arm_poses is not None:
                q_xyzw_r = self.arm_r.arm_poses[3:]
                f_raw_r = self.R_sensor_ee @ hand_force[6:9]
                t_raw_r = self.R_sensor_ee @ hand_force[9:12]
                
                f_comp_r, t_comp_r = self.ft_comp_r.compensate(f_raw_r, t_raw_r, q_xyzw_r)
                
                # Apply threshold filtering after compensation
                f_comp_r = np.where(np.abs(f_comp_r) < self.force_threshold, 0, f_comp_r)
                t_comp_r = np.where(np.abs(t_comp_r) < self.torque_threshold, 0, t_comp_r)
                
                ft_msg_r = WrenchStamped()
                ft_msg_r.header.stamp = self.get_clock().now().to_msg()
                ft_msg_r.header.frame_id = "Link7_r"
                ft_msg_r.wrench.force.x = f_comp_r[0]
                ft_msg_r.wrench.force.y = f_comp_r[1]
                ft_msg_r.wrench.force.z = f_comp_r[2]
                ft_msg_r.wrench.torque.x = t_comp_r[0]
                ft_msg_r.wrench.torque.y = t_comp_r[1]
                ft_msg_r.wrench.torque.z = t_comp_r[2]
                self.pub_ft_r.publish(ft_msg_r)

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser(description="G1 Node with F/T filtering")
    # Thresholds prevent sensor noise and gravity error from causing unwanted robot drift
    parser.add_argument('--force-threshold', type=float, default=5.0,
                        help='Force threshold for filtering (N)')
    parser.add_argument('--torque-threshold', type=float, default=1.0,
                        help='Torque threshold for filtering (N*m)')
    
    # Use rclpy to remove ros args for clean parsing
    non_ros_args = rclpy.utilities.remove_ros_args(sys.argv[1:])
    parsed_args = parser.parse_args(non_ros_args)
    
    print("Initializing Robot...")
    robot = Robot()
    time.sleep(5)  # Wait for robot connection
    print("Robot initialized.")
    
    node = G1Node(robot, force_threshold=parsed_args.force_threshold, 
                  torque_threshold=parsed_args.torque_threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
