#!/usr/bin/env python3

# Heecheol Kim | heecheolkim@microsoft.com | MSRA Tokyo | 2025-12-23
# Main g1_proxy launch file:
# - Starts ros2_control nodes for left and right arms
# - Launches controller spawners

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('g1_proxy')
    
    # Generate robot_description for left arm
    robot_description_left = ParameterValue(
        Command(
            [
                'xacro ', 
                PathJoinSubstitution([pkg_share, 'urdf', 'g1_left_arm.urdf'])
            ]
        ),
        value_type=str
    )
    
    # Generate robot_description for right arm
    robot_description_right = ParameterValue(
        Command(
            [
                'xacro ', 
                PathJoinSubstitution([pkg_share, 'urdf', 'g1_right_arm.urdf'])
            ]
        ),
        value_type=str
    )
    
    # Controller config files
    config_dir = os.path.join(
        get_package_share_directory('g1_proxy'),
        'config'
    )

    controllers_yaml = os.path.join(config_dir, 'controllers.yaml')

    # Left Arm Controller Manager node
    # Loads ros2_control configuration from g1_left_arm.urdf and g1_right_arm.urdf:
    #   <ros2_control name="ProxyRobotSystem" type="system">
    #     <hardware>
    #       <plugin>g1_proxy/ProxySystem</plugin>
    #       <param name="arm_id">left_arm</param>
    #     </hardware>

    control_node_left = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='left_arm',  # Namespace required for dual-arm setup
        parameters=[
            {
                'robot_description': robot_description_left,
            },
            controllers_yaml,
        ],
        output='screen',
        # Topic name remappings: Unify topic names published by cartesian_*_controller
        # so all active controllers receive the same target (target_frame_left)
        # and we can distinguish left/right arm topics
        remappings=[
            ('cartesian_motion_controller/target_frame', 'target_frame_left'),
            ('cartesian_compliance_controller/target_frame', 'target_frame_left'),
            ('cartesian_force_controller/target_wrench', 'target_wrench_left'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench_left'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench_left'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench_left'),
            ('force_torque_sensor_broadcaster/wrench', 'ft_sensor_wrench_left'),
        ]
    )

    # Right Arm Controller Manager node
    control_node_right = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='right_arm',
        parameters=[
            {
                'robot_description': robot_description_right,
            },
            controllers_yaml,
        ],
        output='screen',
        remappings=[
            ('cartesian_motion_controller/target_frame', 'target_frame_right'),
            ('cartesian_compliance_controller/target_frame', 'target_frame_right'),
            ('cartesian_force_controller/target_wrench', 'target_wrench_right'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench_right'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench_right'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench_right'),
            ('force_torque_sensor_broadcaster/wrench', 'ft_sensor_wrench_right'),
        ]
    )

    # Left Arm Spawners
    left_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/left_arm/controller_manager'],
    )

    left_force_torque_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['force_torque_sensor_broadcaster', '--controller-manager', '/left_arm/controller_manager'],
    )
    
    # Spawn multiple cartesian_controller types:
    # - compliance_controller: initially active
    # - force and motion controllers: initially inactive
    # View status: ros2 control list_controllers --controller-manager /left_arm/controller_manager
    # Switch controllers:
    #   ros2 control switch_controllers --controller-manager /left_arm/controller_manager \
    #     --deactivate cartesian_compliance_controller \
    #     --activate cartesian_motion_controller

    left_cartesian_compliance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_compliance_controller', '--controller-manager', '/left_arm/controller_manager'],
    )

    left_cartesian_force_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_force_controller', '--controller-manager', '/left_arm/controller_manager', '--inactive'],
    )

    left_cartesian_motion_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_motion_controller', '--controller-manager', '/left_arm/controller_manager', '--inactive'],
    )

    # Right Arm Spawners
    right_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/right_arm/controller_manager'],
    )

    right_force_torque_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['force_torque_sensor_broadcaster', '--controller-manager', '/right_arm/controller_manager'],
    )

    right_cartesian_compliance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_compliance_controller', '--controller-manager', '/right_arm/controller_manager'],
    )

    right_cartesian_force_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_force_controller', '--controller-manager', '/right_arm/controller_manager', '--inactive'],
    )

    right_cartesian_motion_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_motion_controller', '--controller-manager', '/right_arm/controller_manager', '--inactive'],
    )
    
    return LaunchDescription([
        # Left Arm
        control_node_left,
        left_joint_state_broadcaster_spawner,
        left_force_torque_sensor_broadcaster_spawner,
        left_cartesian_compliance_controller_spawner,
        left_cartesian_force_controller_spawner,
        left_cartesian_motion_controller_spawner,
        # Right Arm
        control_node_right,
        right_joint_state_broadcaster_spawner,
        right_force_torque_sensor_broadcaster_spawner,
        right_cartesian_compliance_controller_spawner,
        right_cartesian_force_controller_spawner,
        right_cartesian_motion_controller_spawner,
    ])
