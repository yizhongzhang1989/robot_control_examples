#!/usr/bin/env python3

# Heecheol Kim | heecheolkim@microsoft.com | MSRA Tokyo | 2025-12-23
# Performs F/T sensor calibration using tools from
# cartesian_controllers_ros2/cartesian_controller_tools/cartesian_controller_tools/ft_compensator.py
# G1 specific implementation.
# Usage: python g1_proxy/scripts/g1_online_ft_calibrator.py --arm left
# Note: Calibration poses must be pre-defined in g1_proxy/config/calibration_poses.yaml

import yaml
import numpy as np
import json
import os
import time
from a2d_sdk.robot import RobotDds as Robot
from ruckig import Ruckig, InputParameter, OutputParameter

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../cartesian_controllers_ros2/cartesian_controller_tools'))
from cartesian_controller_tools.ft_compensator import FTCalibrationSolver

from g1_node import G1FKSolver, G1ArmController


class G1OnlineFTCalibrator:
    def __init__(self, robot, arm_side="left"):
        self.arm_side = arm_side
        self.cfg = {
            'poses_file': 'config/calibration_poses.yaml',
            'output_file': f'config/ft_calibration_result_{arm_side}.json',
            'arrival_threshold_joint': 0.01,
            'settle_time': 1.0,
            'collect_time': 1.,
        }
        
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for key in ['poses_file', 'output_file']:
            if not os.path.isabs(self.cfg[key]):
                self.cfg[key] = os.path.join(pkg_root, self.cfg[key])
        
        with open(self.cfg['poses_file'], 'r') as f:
            poses_data = yaml.safe_load(f)
            poses_key = f'calibration_poses_{arm_side}'
            self.poses = poses_data.get(poses_key, [])
        
        self.controller = G1ArmController(robot, arm_side=arm_side)
        # Use conservative limits for calibration
        self.controller.set_limits(1.0, 2.0, 5.0)
        
        self.R_sensor_ee = np.array([[0, 1, 0], [-1, 0, 0], 
                                     [0, 0, 1]])
        self.collected_data = []
        self.pose_idx = 0
        self.state = 'SEND_POSE'
        self.state_start_time = 0

    def control_loop(self):
        # Force/torque data collection loop
        if not self.controller.update_state():
            return False
        
        curr_arm = self.controller.current_left + self.controller.current_right
        curr_arm_joints = curr_arm[:7] if self.controller.arm_side == 'l' else curr_arm[7:14]
        now = time.time()
        
        if self.state == 'SEND_POSE':
            if self.pose_idx >= len(self.poses):
                self.save_result()
                self.state = 'DONE'
                return True
            self.target_joints = self.poses[self.pose_idx]
            self.state = 'WAITING_ARRIVAL'
            
        elif self.state == 'WAITING_ARRIVAL':
            if np.linalg.norm(np.array(curr_arm_joints) - self.target_joints) < self.cfg['arrival_threshold_joint']:
                self.state, self.state_start_time = 'SETTLING', now
            self.controller.move(self.target_joints)
        elif self.state == 'SETTLING':
            self.controller.move(self.target_joints)
            if now - self.state_start_time > self.cfg['settle_time']:
                self.state, self.state_start_time, self.temp_data = 'COLLECTING', now, []
                
        elif self.state == 'COLLECTING':
            self.controller.move(self.target_joints)
            arm_poses, h_force = self.controller.arm_poses, self.controller.get_hand_force()
            
            if h_force is not None and arm_poses is not None:
                # Apply R_sensor_ee transformation (same as g1_node)
                f_raw = self.R_sensor_ee @ h_force[:3]
                t_raw = self.R_sensor_ee @ h_force[3:6]
                
                self.temp_data.append({
                    'q': arm_poses[3:7],
                    'f': f_raw,
                    't': t_raw
                })
            if now - self.state_start_time > self.cfg['collect_time']:
                self.collected_data.append({k: np.mean([d[k] for d in self.temp_data], 0) for k in ['q', 'f', 't']})
                self.pose_idx += 1
                self.state = 'SEND_POSE'

        return self.state == 'DONE'

    def save_result(self):
        result = FTCalibrationSolver.solve(self.collected_data)
        if result:
            # Convert collected_data for JSON serialization
            collected_data_serializable = []
            for d in self.collected_data:
                collected_data_serializable.append({
                    'q': d['q'].tolist() if isinstance(d['q'], np.ndarray) else d['q'],
                    'f': d['f'].tolist() if isinstance(d['f'], np.ndarray) else d['f'],
                    't': d['t'].tolist() if isinstance(d['t'], np.ndarray) else d['t']
                })
            
            # Add collected data to result
            result['collected_data'] = collected_data_serializable
            
            with open(self.cfg['output_file'], 'w') as f:
                json.dump(result, f, indent=2)
            m = result['calibration']['mass']
            print(f"Calibration saved to {self.cfg['output_file']}. Mass: {m:.4f} kg")
            print(f"Collected {len(self.collected_data)} data points")

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description="G1 Online FT Calibrator")
    parser.add_argument('--arm', type=str, default='left', choices=['left', 'right'],
                        help='Which arm to calibrate (left or right)')
    parser.add_argument('--force-threshold', type=float, default=5.0,
                        help='Force threshold for filtering (N)')
    parser.add_argument('--torque-threshold', type=float, default=1.0,
                        help='Torque threshold for filtering (N*m)')
    args = parser.parse_args()
    
    print("Initializing Robot...")
    robot = Robot()
    time.sleep(5)  # Wait 5 seconds after robot startup for initialization
    print("Robot initialized.")
    
    cal = G1OnlineFTCalibrator(robot, arm_side=args.arm)
    while not cal.control_loop():
        time.sleep(0.01)

if __name__ == '__main__':
    main()
