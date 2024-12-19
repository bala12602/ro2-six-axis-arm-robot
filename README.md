# 6-DOF Robot Arm - ROS2 Control Package

## Overview
This package provides ROS2 control and visualization capabilities for a 6-DOF robotic arm. It includes motion planning, kinematics, control interfaces, and simulation support.

## Features
- Complete 6-DOF arm control system
- Forward and Inverse Kinematics
- Motion Planning using MoveIt2
- URDF/XACRO model description
- Gazebo simulation support
- RViz visualization
- Hardware interface for real robot control
- Joint trajectory execution
- Custom service and action interfaces

## Prerequisites
- ROS2 Humble/Iron
- Ubuntu 22.04
- Required ROS2 packages:
  ```bash
  ros2_control
  ros2_controllers
  moveit2
  gazebo_ros2_control
  xacro
  joint_state_publisher
  robot_state_publisher