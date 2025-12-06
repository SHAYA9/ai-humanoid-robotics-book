---
sidebar_position: 6
title: 'Practical 2: Digital Twin'
---

# Practical 2: Building a Digital Twin

In this practical exercise, we'll bring together the concepts from this module to create a **digital twin** of a simple robot in a simulated environment. A digital twin is a virtual model of a physical object, used to simulate its behavior and monitor its state.

## Objectives

1.  **Create a URDF model** of a simple robotic arm.
2.  **Build a simple environment** in Gazebo with a few objects for the arm to interact with.
3.  **Integrate the robot and environment** in a Gazebo simulation.
4.  **Control the robot's joints** using ROS 2 topics.
5.  **Visualize the robot's sensor data** in RViz.

## Steps

*   **Step 1: The URDF.** Define the links and joints of your robotic arm in a URDF file. You can use simple geometric shapes for the links.
*   **Step 2: The Environment.** Create a new world file in Gazebo and add a ground plane and a few simple objects like cubes and spheres.
*   **Step 3: Launching the Simulation.** Write a ROS 2 launch file that starts Gazebo with your custom world and spawns your URDF model.
*   **Step 4: Joint Control.** Use the `ros2 topic pub` command to send messages to the `/joint_states` topic and move the joints of your robot.
*   **Step 5: Visualization.** Launch RViz and add displays for the robot model and any simulated sensors (e.g., a camera).

This exercise will give you a hands-on understanding of how the different components of a robotics simulation fit together.
