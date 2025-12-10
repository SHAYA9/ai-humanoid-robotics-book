---
sidebar_position: 3
title: Isaac ROS
---

# Isaac ROS: Hardware-Accelerated ROS Packages

**Isaac ROS** is a collection of ROS 2 packages that are optimized to run on NVIDIA's Jetson and GPU platforms. These packages provide hardware acceleration for common robotics tasks, enabling real-time performance that would be difficult to achieve with a CPU alone.

## Key Areas of Acceleration

*   **Perception:** Isaac ROS includes packages for a wide range of perception tasks, such as:
    *   **Stereo Depth Perception:** Generating depth images from a stereo camera pair.
    *   **Object Detection and Recognition:** Using deep learning models to identify and classify objects in the environment.
    *   **Visual SLAM (Simultaneous Localization and Mapping):** Building a map of the environment and tracking the robot's position within it.
*   **Navigation:** Isaac ROS provides GPU-accelerated versions of key components of the ROS 2 Navigation stack (Nav2), such as the costmap generation.
*   **Manipulation:** There are packages for tasks like grasp planning and motion planning that can take advantage of GPU acceleration.

## The Power of NITROS

Under the hood, Isaac ROS uses **NITROS (NVIDIA Isaac Transport for ROS)**, a framework that optimizes the data transport layer in ROS 2. By leveraging zero-copy memory access and other techniques, NITROS minimizes the overhead of passing data between different ROS nodes, which is crucial for high-throughput perception pipelines.
