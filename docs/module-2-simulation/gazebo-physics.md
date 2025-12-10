---
sidebar_position: 2
title: Gazebo Physics
---

# Gazebo Physics

**Gazebo** is a powerful, open-source 3D robotics simulator. At its core is a robust physics engine that allows you to simulate the dynamics of robots and their interactions with the environment with a high degree of realism.

## The Role of the Physics Engine

Gazebo uses a pluggable physics engine architecture, with **ODE (Open Dynamics Engine)** being the default. These engines are responsible for calculating:

-   **Dynamics:** The motion of objects under the influence of forces and torques. This includes gravity, friction, and collisions.
-   **Kinematics:** The motion of the robot's joints and links, taking into account joint limits and constraints.
-   **Collision Detection:** Detecting when and where objects in the simulation come into contact.

## SDF: The Simulation Description Format

Gazebo uses the **Simulation Description Format (SDF)** to describe everything in the simulation world, from the robots to the environment. SDF is an XML format that allows you to specify:

-   The physical properties of objects, such as mass, inertia, and friction.
-   The visual properties of objects, such as their shape, color, and texture.
-   The properties of the physics engine, such as gravity and the simulation time step.

Here is a simple example of an SDF file for a box:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Integrating with ROS 2

Gazebo is tightly integrated with ROS 2. The `gazebo_ros` package provides a set of plugins that allow you to:

-   Spawn URDF and SDF models into the simulation.
-   Control the joints of your robot using ROS 2 messages.
-   Simulate a wide variety of sensors and publish their data to ROS 2 topics.

This tight integration allows you to use the same ROS 2 code to control both your simulated robot and your physical robot, which is a key principle of the digital twin concept.

In the [Practical 2: Digital Twin](./practical-2-digital-twin.md), we will create a Gazebo simulation of our humanoid robot and learn how to control it with ROS 2.
