---
sidebar_position: 1
---

# 7.1 What is ROS 2?

Welcome to the Robot Operating System (ROS 2), the communication backbone of modern robotics. While not a traditional operating system like Windows or Linux, ROS 2 provides a powerful framework of libraries and tools to help you build complex robot applications. Think of it less as an OS and more as a **robotic nervous system**—a standardized way for different parts of your robot to communicate and work together.

### From ROS 1 to ROS 2: A New Architecture

The original ROS (now called ROS 1) revolutionized robotics by making it easier to reuse code and integrate different hardware and software components. However, it was primarily designed for academic research on a single robot with a stable network connection.

ROS 2 was redesigned from the ground up to meet the demands of commercial and industrial applications, focusing on:

-   **Multi-Robot Systems:** Natively supports communication between multiple robots.
-   **Real-Time Control:** Enables high-performance, deterministic control loops.
-   **Unreliable Networks:** Handles challenging network conditions, like Wi-Fi dropouts.
-   **Production Environments:** Emphasizes security, robustness, and long-term support (LTS) releases.

The key technology behind this is the **Data Distribution Service (DDS)**, an industry-standard protocol for real-time data exchange that ROS 2 is built upon.

### Why a "Robotic Nervous System"?

A robot is a complex system of sensors, actuators, and computers.
-   **Sensors** (eyes, ears) perceive the world.
-   **Actuators** (muscles) perform actions.
-   **Computers** (the brain) process information and make decisions.

ROS 2 acts as the nervous system that connects them all. A camera driver can publish images, a perception algorithm can process them to find an object, a planner can decide how to pick it up, and a controller can execute the motion. Each of these components is a separate program, and ROS 2 allows them to communicate seamlessly as if they were one.

### Core Philosophy

The design of ROS 2 is guided by a few core principles:

-   **Distributed and Modular:** Systems are composed of many small, independent programs called **nodes**. A node might be a camera driver, a motion planner, or a motor controller. If one node crashes, the rest of the system can keep running.
-   **Peer-to-Peer:** There is no central "master" node. Nodes discover and talk to each other directly, making the system more resilient.
-   **Language Agnostic:** You can write nodes in C++, Python, and other languages, and they can all communicate with each other.
-   **Open Source:** ROS 2 is developed and maintained by a global community, making it a free and powerful platform for everyone from hobbyists to large corporations.

In the next sections, we will dive into the core concepts that make this nervous system tick: nodes, topics, services, and actions.
