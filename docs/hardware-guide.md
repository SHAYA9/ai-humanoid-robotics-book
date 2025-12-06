---
sidebar_position: 4
title: Hardware Guide
---

# Detailed Hardware Guide

This guide provides detailed specifications and recommendations for the hardware used in this course. While we provide a cloud alternative, having access to physical hardware will provide the best learning experience.

## Development Workstation

The development workstation is the a crucial component of your robotics lab. It will be used for simulation, AI model training, and programming.

-   **OS:** Ubuntu 22.04 LTS is the recommended operating system, as it is the primary target for ROS 2 Humble. We provide instructions for a dual-boot setup with Windows.
-   **GPU:** An NVIDIA RTX 4070 Ti or 4090 is required for running NVIDIA Isaac Sim and training deep learning models.
-   **RAM:** 64GB of RAM is recommended for running complex simulations and training large models.

For a complete parts list and build guide, see the [Hardware Setup](./resources/hardware-setup.md) section.

## Edge Computing Kit

The edge computing kit is the "brain" of your robot. It will run the ROS 2 nodes, process sensor data, and execute the AI models.

-   **NVIDIA Jetson Orin Nano:** This powerful and compact single-board computer is ideal for robotics applications.
-   **Intel RealSense D435i:** This depth camera provides high-quality color, depth, and infrared streams, as well as an integrated IMU.
-   **Power Management:** A reliable power management system is crucial for a mobile robot. We provide several recommendations in the hardware setup guide.

## Robot Platforms

This course is designed to be adaptable to a variety of robot platforms. We provide custom ROS 2 interfaces for the following recommended robots:

-   **Unitree Go2:** An advanced quadruped robot that serves as an excellent platform for learning about locomotion and dynamic control.
-   **Unitree G1:** A state-of-the-art humanoid robot for those who want to work with a bipedal platform.
-   **Hiwonder TonyPi Pro:** A more budget-friendly humanoid robot kit that is great for getting started.

You are also welcome to use your own custom-built robot. We provide guidelines for creating a URDF model and integrating it with ROS 2.
