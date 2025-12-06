---
sidebar_position: 3
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

Now that we have a high-level overview of the ROS 2 architecture, let's dive deeper into the core components that make it work: nodes, topics, services, and actions.

## Nodes

A **node** is the fundamental building block of a ROS 2 system. It is a process that performs a specific, well-defined task. For example, in a humanoid robot, you might have separate nodes for:

-   Controlling the motors in the left leg.
-   Reading data from the IMU sensor.
-   Processing images from the head camera.
-   Planning the robot's path.

Each node can be written in a different programming language (Python and C++ are the most common) and can be run on a different computer, as long as they are on the same network. This distributed nature is a key feature of ROS 2.

## Topics

**Topics** are the primary mechanism for asynchronous, publish/subscribe communication between nodes. They are named buses that nodes can use to exchange messages.

-   A node can **publish** messages to a topic.
-   A node can **subscribe** to a topic to receive messages.

This is a one-to-many or many-to-many communication pattern. For example, the camera node can publish images to an `/camera/image_raw` topic, and multiple nodes (e.g., an object detection node and a logging node) can subscribe to this topic to receive the images.

## Services

**Services** are used for synchronous, request/response communication. This is a one-to-one communication pattern, where a client node sends a request to a server node and waits for a response.

Services are useful for tasks that have a clear start and end, and where the client needs a direct confirmation that the task has been completed. For example:

-   A service to turn a camera on or off.
-   A service to calculate the distance between two points.
-   A service to save the current robot state to a file.

## Actions

**Actions** are used for long-running, asynchronous tasks that provide feedback. This is a more complex communication pattern that is ideal for tasks that take a significant amount of time to complete.

An action has three components:

1.  **Goal:** The client sends a goal to the action server (e.g., "walk to the kitchen").
2.  **Feedback:** The action server provides regular feedback to the client on the progress of the goal (e.g., "I have walked 5 meters").
3.  **Result:** When the goal is complete, the action server sends a final result to the client (e.g., "I have reached the kitchen").

Actions are perfect for tasks like navigation, manipulation, and other complex behaviors. In the next section, we'll learn how to write our own ROS 2 nodes using the [`rclpy`](./rclpy-python-agents.md) library.
