---
sidebar_position: 1
title: "Module 1: The Robotic Nervous System (ROS 2)"
---

# Module 1: The Robotic Nervous System (ROS 2)

Welcome to Module 1. In this section, we will explore the **Robot Operating System (ROS 2)**, the middleware that will serve as the nervous system for our humanoid robots.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It is a set of libraries and tools that help you build complex robot applications, from the low-level drivers to the high-level AI. It is not a traditional operating system, but rather a meta-operating system that provides services like hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

## Key Topics

In this module, we will cover the following topics:

-   **ROS 2 Architecture:** The fundamental concepts of the ROS 2 graph, including nodes, topics, services, and actions.
-   **Nodes, Topics, & Services:** The building blocks of a ROS 2 application.
-   **`rclpy`:** The Python client library for ROS 2. We will use `rclpy` to write our own ROS 2 nodes.
-   **URDF:** The Unified Robot Description Format, an XML format for representing a robot model. We will learn how to create a URDF for a humanoid robot.
-   **Humanoid Control:** The basics of controlling a humanoid robot with ROS 2, including joint state publishing and trajectory control.

## Learning Objectives

By the end of this module, you will be able to:

-   Understand the core concepts of ROS 2.
-   Write your own ROS 2 nodes in Python.
-   Create a URDF model of a simple humanoid robot.
-   Control a simulated humanoid robot in Gazebo.

Let's get started with the [ROS 2 Architecture](./ros2-architecture.md).
