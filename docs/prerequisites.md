---
sidebar_position: 2
title: Prerequisites
---

# Prerequisites

To successfully complete this course, you will need a combination of hardware and software. We have designed the curriculum to be as accessible as possible, with a cloud-based lab alternative for those who do not have access to the recommended hardware.

## Hardware Requirements

### Development Workstation

-   **OS:** Ubuntu 22.04 LTS
-   **CPU:** Intel Core i7-12700K or AMD Ryzen 7 7700X
-   **GPU:** NVIDIA GeForce RTX 4070 Ti or RTX 4090
-   **RAM:** 64GB DDR5
-   **Storage:** 2TB NVMe SSD

A detailed guide to setting up your workstation can be found in the [Hardware Setup](./resources/hardware-setup.md) section.

### Edge Computing & Robotics Kit

-   **SBC:** NVIDIA Jetson Orin Nano Developer Kit
-   **Camera:** Intel RealSense Depth Camera D435i
-   **IMU:** A high-quality IMU sensor (specific model recommendations are in the hardware guide).
-   **Robot:** While the course is designed to be platform-agnostic, we recommend one of the following:
    -   Unitree Go2 (quadruped)
    -   Unitree G1 (humanoid)
    -   Hiwonder TonyPi Pro (budget-friendly humanoid)

## Software Requirements

-   **ROS 2 Humble Hawksbill:** The official ROS 2 LTS release.
-   **Gazebo 11:** The standard for robotics simulation.
-   **Unity:** For advanced visualization and simulation.
-   **NVIDIA Isaac Sim & Isaac ROS:** You will need to apply for access through the NVIDIA Developer Program.
-   **Python 3.10+** and **C++17**

## Cloud Lab Alternative

For students who do not have access to the recommended hardware, we provide a cloud-based lab environment.

-   **Cloud Provider:** AWS
-   **Instance Type:** g5.2xlarge
-   **Services:** Omniverse Cloud, AWS RoboMaker

A complete guide to setting up the cloud lab can be found in the [Cloud Lab](./resources/cloud-lab.md) section. Please be aware of the potential costs associated with cloud computing.
