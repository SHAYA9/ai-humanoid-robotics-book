---
id: glossary
title: "Glossary of Terms"
sidebar_label: "Glossary"
description: "Comprehensive glossary of terms used in Physical AI and Humanoid Robotics"
tags: [glossary, terminology, definitions, robotics, ai]
---

# Glossary of Terms

## A

### Action (ROS 2)
A communication pattern in ROS 2 for long-running tasks with feedback, cancellation, and result capabilities. Used for operations like navigation or manipulation that take time to complete.

### Actuator
A device that converts electrical signals into physical motion. In humanoid robots, actuators are typically electric motors (servos) that move joints.

### API (Application Programming Interface)
A set of rules and protocols that allows different software applications to communicate with each other.

### Autonomous Navigation
The ability of a robot to move through an environment without human intervention, using sensors to perceive obstacles and plan paths.

## B

### Bipedal Locomotion
Walking using two legs, characteristic of humanoid robots. Requires sophisticated balance and control algorithms.

### Bundle Adjustment
An optimization problem in computer vision and robotics that refines camera parameters and 3D point coordinates to minimize reprojection error.

## C

### CAD (Computer-Aided Design)
Software used to create precision drawings or technical illustrations, essential for robot design and simulation.

### Calibration
The process of adjusting sensor measurements to match known values, ensuring accuracy in perception systems.

### CAN Bus (Controller Area Network)
A robust vehicle bus standard designed to allow microcontrollers and devices to communicate with each other in applications without a host computer.

### Capstone Project
A culminating academic and intellectual experience that integrates knowledge and skills learned throughout the course.

### CNN (Convolutional Neural Network)
A class of deep neural networks most commonly applied to analyzing visual imagery.

### Computer Vision
A field of artificial intelligence that enables computers to derive meaningful information from digital images, videos, and other visual inputs.

### Controller
A device or algorithm that manages the behavior of a system. In robotics, controllers regulate motor positions, velocities, or torques.

## D

### DDS (Data Distribution Service)
A middleware protocol and API standard for data-centric connectivity used in ROS 2 for communication between nodes.

### Degrees of Freedom (DOF)
The number of independent parameters that define the configuration of a mechanical system. A humanoid robot typically has 20-30 DOF.

### Digital Twin
A virtual representation of a physical object or system that spans its lifecycle, is updated from real-time data, and uses simulation for decision making.

### DOF (Degrees of Freedom)
See Degrees of Freedom.

## E

### Edge AI
Running AI algorithms locally on a hardware device (like Jetson) rather than in the cloud, enabling real-time processing and privacy.

### Embodied AI
AI systems that learn by interacting with a physical environment through sensors and actuators, rather than just processing data.

### End Effector
The device at the end of a robotic arm, designed to interact with the environment. Common types: grippers, suction cups, tools.

### Ephemeral Port (ROS 2)
Temporary network ports used for ROS 2 communication that change between sessions.

## F

### FK (Forward Kinematics)
Calculating the position and orientation of the end effector given the joint angles of a robot.

### FPS (Frames Per Second)
A measure of how many unique consecutive images (frames) a system can produce each second.

### Frame (ROS 2)
A coordinate system in ROS 2, used for transformations between different parts of a robot.

## G

### Gait
A pattern of movement of the limbs of animals, including humans, during locomotion. In robotics, refers to walking patterns.

### Gazebo
An open-source 3D robotics simulator that accurately simulates populations of robots in complex indoor and outdoor environments.

### GPT (Generative Pre-trained Transformer)
A type of large language model developed by OpenAI, used in this course for natural language understanding in robotics.

### GPU (Graphics Processing Unit)
A specialized processor originally designed to accelerate graphics rendering, now widely used for AI and deep learning computations.

### Gripper
An end effector designed to grasp and manipulate objects, analogous to a human hand.

## H

### HRI (Human-Robot Interaction)
The study of interactions between humans and robots, including communication, safety, and collaboration.

### Humanoid Robot
A robot with its body shape built to resemble the human body, typically with a torso, head, two arms, and two legs.

## I

### IK (Inverse Kinematics)
Calculating the joint angles required to achieve a desired position and orientation of the end effector.

### IMU (Inertial Measurement Unit)
An electronic device that measures and reports a body's specific force, angular rate, and sometimes magnetic field.

### Isaac Sim
NVIDIA's robotics simulation platform built on Omniverse, providing photorealistic, physically accurate virtual environments.

### Isaac ROS
NVIDIA's collection of hardware-accelerated ROS 2 packages for perception and AI inference.

## J

### Jacobian Matrix
In robotics, a matrix representing the relationship between joint velocities and end-effector velocities.

### Jetson
NVIDIA's series of embedded computing boards designed for AI at the edge, commonly used in robotics.

### Joint
A connection between two robot links that allows relative motion. Types include: revolute (rotational), prismatic (linear), and spherical.

## K

### Kinematics
The study of motion without considering the forces that cause it. In robotics: forward kinematics and inverse kinematics.

### Kinesthetic Teaching
A robot programming method where a human physically guides the robot through a task, which the robot then remembers and repeats.

## L

### LiDAR (Light Detection and Ranging)
A remote sensing method that uses light in the form of a pulsed laser to measure ranges.

### LLM (Large Language Model)
AI models trained on massive amounts of text data that can generate human-like text and understand language context.

### Locomotion
The ability to move from one place to another. In humanoid robots: walking, running, climbing.

## M

### Manipulator
The arm of a robot, consisting of links and joints, used to manipulate objects in the environment.

### Middleware
Software that provides common services and capabilities to applications outside of what's offered by the operating system. ROS 2 is robotics middleware.

### Motion Planning
The process of breaking down a desired movement task into discrete motions that satisfy movement constraints.

### MoveIt 2
The ROS 2 version of MoveIt, a software framework for mobile manipulation, incorporating state-of-the-art algorithms for motion planning.

## N

### Nav2
The ROS 2 navigation stack for mobile robots, providing path planning, localization, and obstacle avoidance.

### Node (ROS 2)
A process that performs computation in ROS 2. Nodes communicate with each other using topics, services, and actions.

### NVIDIA Omniverse
A platform for connecting 3D tools and developing custom applications, serving as the foundation for Isaac Sim.

## O

### Object Detection
A computer vision technique for identifying and locating objects in images or video.

### Occupancy Grid
A representation of the environment where space is divided into cells, each storing the probability of being occupied.

### Odometry
The use of data from motion sensors to estimate change in position over time.

### Omniverse
See NVIDIA Omniverse.

### OpenCV (Open Source Computer Vision Library)
An open-source computer vision and machine learning software library.

## P

### Package (ROS 2)
The primary unit for organizing software in ROS 2, containing nodes, libraries, configuration files, etc.

### Parameter (ROS 2)
Configuration values that can be set at runtime and are accessible to nodes.

### Path Planning
The process of finding a collision-free path from a start to a goal position.

### Perception
The process of using sensors to obtain information about the environment.

### Physical AI
AI systems that interact with and understand the physical world, combining perception, reasoning, and action.

### PID Controller (Proportional-Integral-Derivative)
A control loop mechanism employing feedback widely used in industrial control systems and robotics.

### Point Cloud
A set of data points in space, often produced by 3D scanners or depth cameras.

### Pose
The position and orientation of an object in space, typically represented as (x, y, z, roll, pitch, yaw).

### Publisher (ROS 2)
A node that sends messages on a topic.

## Q

### QoS (Quality of Service)
In ROS 2, policies that control how messages are delivered between nodes, including reliability, durability, and deadline.

### Quaternion
A mathematical representation of 3D orientation using four numbers, avoiding gimbal lock issues of Euler angles.

## R

### Real-time System
A computer system that must respond to events within a guaranteed time frame, critical for robot control.

### Reinforcement Learning
A machine learning paradigm where an agent learns to make decisions by taking actions in an environment to maximize cumulative reward.

### ROS (Robot Operating System)
An open-source robotics middleware suite. ROS 2 is the second generation.

### ROS 2
The second generation of ROS, featuring real-time capabilities, multiple DDS implementations, and improved security.

### rclpy
The ROS 2 client library for Python.

### RGB-D
Color (RGB) plus Depth information, typically from cameras like Intel RealSense.

### RTAB-Map (Real-Time Appearance-Based Mapping)
A RGB-D SLAM approach with loop closure detection.

### RTX
NVIDIA's line of graphics cards with ray tracing capabilities, required for Isaac Sim.

## S

### SDK (Software Development Kit)
A collection of software development tools in one installable package.

### Service (ROS 2)
A communication pattern where a client sends a request and receives a response, used for one-time computations.

### Servo Motor
A rotary or linear actuator that allows for precise control of angular or linear position, velocity, and acceleration.

### Simulation
Creating a virtual model of a real-world system to study its behavior.

### Sim-to-Real
Transferring skills learned in simulation to the real world.

### SLAM (Simultaneous Localization and Mapping)
The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

### Subscriber (ROS 2)
A node that receives messages from a topic.

## T

### TCP (Tool Center Point)
The point on a robot tool whose position and orientation define the coordinates of the tool.

### TF2 (Transform Library)
The ROS 2 library for keeping track of multiple coordinate frames over time.

### Thread
The smallest sequence of programmed instructions that can be managed independently by a scheduler.

### Time Synchronization
Ensuring different system components share a common understanding of time, critical for sensor fusion.

### Topic (ROS 2)
A named bus over which nodes exchange messages in a publish-subscribe pattern.

### Torque
A measure of the force that can cause an object to rotate about an axis, important for motor control.

## U

### Ubuntu
A Linux distribution based on Debian, commonly used in robotics development.

### Unity
A cross-platform game engine used in this course for high-fidelity visualization and human-robot interaction simulation.

### URDF (Unified Robot Description Format)
An XML format for representing a robot model in ROS, describing links, joints, and their relationships.

### USB (Universal Serial Bus)
An industry standard for cables, connectors, and protocols for connection, communication, and power supply between computers and devices.

## V

### VLA (Vision-Language-Action)
Models that integrate visual perception, language understanding, and physical action for robotics.

### VSLAM (Visual SLAM)
SLAM using visual sensors (cameras) as the primary source of information.

## W

### Waypoint
An intermediate point or place on a route or line of travel, used in path planning.

### Whisper
OpenAI's automatic speech recognition system, used in this course for voice command recognition.

### Workspace (ROS 2)
A directory containing ROS 2 packages, organized for building and development.

## X

### Xacro (XML Macros)
An XML macro language for URDF files that allows for more readable and maintainable robot descriptions.

## Y

### YAML (YAML Ain't Markup Language)
A human-readable data serialization language often used for configuration files in ROS 2.

### YOLO (You Only Look Once)
A real-time object detection system used in computer vision, particularly YOLOv8 in this course.

## Z

### Zero Moment Point (ZMP)
A concept related to dynamics and control of legged locomotion, important for bipedal robot stability.

### Zero-shot Learning
The ability of a model to correctly make predictions for classes it has never seen during training.

### z-height
The vertical coordinate in a 3D coordinate system, important for manipulation and navigation planning.

---

## Common Acronyms

- **AI:** Artificial Intelligence
- **API:** Application Programming Interface
- **CAD:** Computer-Aided Design
- **CNN:** Convolutional Neural Network
- **CPU:** Central Processing Unit
- **DDS:** Data Distribution Service
- **DOF:** Degrees of Freedom
- **FK:** Forward Kinematics
- **FPS:** Frames Per Second
- **GPU:** Graphics Processing Unit
- **HRI:** Human-Robot Interaction
- **IK:** Inverse Kinematics
- **IMU:** Inertial Measurement Unit
- **LLM:** Large Language Model
- **ML:** Machine Learning
- **PID:** Proportional-Integral-Derivative
- **QoS:** Quality of Service
- **RGB-D:** Red-Green-Blue Depth
- **RL:** Reinforcement Learning
- **ROS:** Robot Operating System
- **SDK:** Software Development Kit
- **SLAM:** Simultaneous Localization and Mapping
- **TCP:** Tool Center Point
- **URDF:** Unified Robot Description Format
- **USB:** Universal Serial Bus
- **VLA:** Vision-Language-Action
- **VSLAM:** Visual SLAM
- **YAML:** YAML Ain't Markup Language
- **ZMP:** Zero Moment Point

---

*This glossary is regularly updated as new terms emerge in the field of Physical AI and Humanoid Robotics. Last updated: January 2024*