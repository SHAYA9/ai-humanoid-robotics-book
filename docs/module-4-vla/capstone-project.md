---
id: capstone-project
title: "Capstone Project: The Autonomous Humanoid"
sidebar_label: "Capstone: Autonomous Humanoid"
description: "Final project integrating all modules: Voice-controlled humanoid with vision, planning, and autonomous navigation"
tags: [capstone, vla, autonomous, humanoid, integration]
---

# Capstone Project: The Autonomous Humanoid

## 🎯 Project Overview

**Objective:** Build a fully autonomous humanoid robot that can:
1. **Hear** voice commands via Whisper
2. **Understand** natural language via LLM planning
3. **See** the environment via computer vision
4. **Plan** navigation via ROS 2
5. **Execute** physical actions via motor control
6. **Manipulate** objects via gripper control

**Project Duration:** 4 weeks (Weeks 10-13)
**Difficulty:** ★★★★★ (Advanced)
**Hardware Required:** See [Hardware Setup](../resources/hardware-setup.md)

## 📋 Project Specifications

### Functional Requirements

| Requirement | Description | Success Criteria |
|------------|-------------|-----------------|
| **Voice Command** | Accept natural language commands | 95% accuracy in quiet environment |
| **LLM Planning** | Convert command to action sequence | Correct sequence for 10 test commands |
| **Object Detection** | Identify target objects | 90% accuracy at 2m distance |
| **Path Planning** | Navigate to target location | Avoid 3+ obstacles successfully |
| **Object Manipulation** | Pick and place objects | Successful grasp of 5 different objects |
| **Safety System** | Emergency stop and collision avoidance | Zero collisions in test environment |

### Technical Stack

```yaml
Perception:
  - Speech: OpenAI Whisper
  - Vision: YOLOv8 (Custom-trained)
  - Depth: Intel RealSense D455
  - IMU: BNO055

Planning:
  - LLM: GPT-4 / Claude 3
  - Task Planner: Custom ROS 2 Node
  - Navigation: Nav2 + MoveIt2

Control:
  - ROS 2: Humble Hawksbill
  - Middleware: rclpy
  - Simulation: NVIDIA Isaac Sim
  - Hardware: Unitree G1 / Jetson Orin

Environment:
  - OS: Ubuntu 22.04 LTS
  - GPU: NVIDIA RTX 4080+ (Training)
  - Edge: Jetson Orin Nano (Deployment)