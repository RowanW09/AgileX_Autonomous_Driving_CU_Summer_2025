# AgileX_Autonomous_Driving_CU_Summer_2025
Clarkson University | Rowan Wysocki, Dr. Chen Liu

This repository documents the Summer 2025 undergraduate research project on autonomous navigation using the AgileX Scout Mini robot with ROS 2. The project integrates real-time SLAM, LiDAR/camera fusion, navigation, and simulation, with deployments in both physical and Gazebo environments.

## Overview
This project focuses on:
- Real-time 2D mapping using SLAM Toolbox
- Path planning and obstacle avoidance via Nav2
- Multi-sensor fusion of Livox LiDAR and Intel RealSense RGB-D
- Autonomous exploration with Explore Lite
- Simulated testing using Gazebo Fortress

## Hardware:
- Scout Mini UGV (AgileX)
- Intel NUC onboard PC (Ubuntu 22.04)
- Livox HAP LiDAR
- Intel RealSense D435 RGB-D Camera

## Core Software Stack:
- ROS 2 Humble
- SLAM Toolbox
- Nav2
- Robot Localization (EKF)
- PointCloud to LaserScan
- Explore Lite
- Custom ROS 2 Node: Multi-sensor point cloud fuser
- Gazebo Fortress for simulation

## Custom Node: Multi-Sensor Point Cloud Fuser
A custom Python node that fuses point clouds from the RealSense RGB-D camera and Livox LiDAR to create a unified perception topic (/fused_points) for improved SLAM and navigation performance.

Key Features:
- Z-axis passthrough filtering
- Voxel downsampling
- TF transformation to base_link

## Simulation Support
Gazebo Fortress + ROS 2 Integration
Simulated Scout Mini and TurtleBot3 environments
Full test cycle: SLAM → Save Map → Nav2 navigation
## Acknowledgments
This project was conducted under the mentorship of Dr. Chen Liu at Clarkson University as part of a Summer 2025 Research Experience.
