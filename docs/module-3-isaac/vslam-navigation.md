---
sidebar_position: 4
title: vSLAM Navigation
---

# vSLAM Navigation with Isaac ROS

**Visual SLAM (vSLAM)** is a technique for Simultaneous Localization and Mapping that uses camera data as its primary input. The Isaac ROS vSLAM package is a powerful, GPU-accelerated implementation that can provide robust and accurate real-time localization and mapping for robots.

## How it Works

The Isaac ROS vSLAM package takes in stereo camera images and IMU data and outputs the robot's pose (position and orientation) and a 3D map of the environment. It's a graph-based SLAM system, which means it builds a graph of keyframes (camera images at specific points in time) and optimizes the relationships between them to produce a globally consistent map.

## Key Features

*   **GPU Acceleration:** The entire vSLAM pipeline is heavily optimized to run on NVIDIA GPUs, allowing for real-time performance even with high-resolution camera data.
*   **Robustness:** The use of stereo vision and IMU data makes the system robust to challenging environments with poor lighting or a lack of distinct visual features.
*   **Integration with Nav2:** The output of the Isaac ROS vSLAM package can be directly fed into the ROS 2 Navigation stack (Nav2) for autonomous navigation.

## Use Cases

vSLAM is particularly well-suited for applications where GPS is unavailable or unreliable, such as indoor navigation, and for robots that are equipped with stereo cameras. It's a key enabling technology for a wide range of autonomous mobile robots (AMRs).
