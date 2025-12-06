---
sidebar_position: 5
title: URDF for Humanoids
---

# URDF for Humanoids

The **Unified Robot Description Format (URDF)** is an XML format for representing a robot model. In ROS 2, URDF is the standard way to describe the physical structure of a robot, including its links, joints, sensors, and visual appearance.

## Core Concepts of URDF

A URDF file is composed of two main components:

-   **Links:** These represent the rigid parts of the robot, such as the torso, arms, and legs. Each link has properties like its mass, inertia, and visual geometry.
-   **Joints:** These connect the links and define how they can move relative to each other. There are several types of joints, including:
    -   `revolute`: A rotational joint, like an elbow.
    -   `prismatic`: A sliding joint, like a piston.
    -   `fixed`: A rigid connection between two links.

## Creating a URDF for a Humanoid Robot

Let's create a simple URDF for a humanoid robot with a torso, a head, and two arms.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.6 0.2" />
      </geometry>
    </visual>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15" />
      </geometry>
    </visual>
  </link>

  <joint name="neck" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0.1 0.1" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1" />
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </visual>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="torso" />
    <child link="right_upper_arm" />
    <origin xyz="0.3 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1" />
  </joint>

</robot>
```

In this example, we have defined:

-   A `torso` link, which is a box.
-   A `head` link, which is a sphere.
-   A `neck` joint that connects the `head` to the `torso` and allows it to rotate around the z-axis.
-   A `right_upper_arm` link, which is a cylinder.
-   A `right_shoulder` joint that connects the `right_upper_arm` to the `torso` and allows it to rotate around the y-axis.

## Visualizing the URDF

You can use **RViz2**, the 3D visualizer for ROS 2, to see your robot model. You will need a `robot_state_publisher` node to read the URDF and publish the robot's state to the `/robot_description` topic.

In the [Practical 1: ROS 2 Setup](./practical-1-ros2-setup.md), you will learn how to create a complete URDF for a humanoid robot and visualize it in RViz2.
