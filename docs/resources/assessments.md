---
id: assessments
title: "Assessments & Grading"
sidebar_label: "Assessments"
description: "Complete assessment structure, rubrics, and evaluation criteria for Physical AI course"
tags: [assessments, grading, rubrics, projects, evaluation]
---

# Assessments & Grading System

## 📊 Overall Course Structure

| Component | Weight | Description | Due Dates |
|-----------|--------|-------------|-----------|
| **Weekly Quizzes** | 15% | 13 quizzes (1% each + 2% bonus) | Every Monday |
| **Module Projects** | 40% | 4 projects (10% each) | Week 3, 6, 9, 12 |
| **Capstone Project** | 30% | Final autonomous humanoid | Week 13 |
| **Participation** | 10% | Lab sessions, discussions | Continuous |
| **Lab Reports** | 5% | 4 hardware lab reports | Week 4, 7, 10, 13 |

**Total: 100%**

## 📝 Assessment 1: Weekly Quizzes (15%)

### Quiz Structure
Each quiz covers the previous week's material and consists of:
- 5 Multiple Choice Questions (2 points each)
- 2 Code Analysis Questions (3 points each)
- 1 Scenario Problem (4 points)

**Total per quiz: 20 points (15 quizzes = 300 points total)**

### Sample Quiz (Week 3: ROS 2 Fundamentals)

```markdown
# Quiz 3: ROS 2 Fundamentals
**Time:** 30 minutes  
**Total Points:** 20

## Part A: Multiple Choice (2 points each)

1. What is the primary purpose of a ROS 2 Node?
   A) To store configuration parameters
   B) To perform computation and communicate
   C) To manage package dependencies
   D) To handle user interface

2. Which command lists all active ROS 2 nodes?
   A) `ros2 node list`
   B) `ros2 list nodes`
   C) `ros2 show nodes`
   D) `ros2 get nodes`

## Part B: Code Analysis (3 points each)

3. Analyze this ROS 2 Python code:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.publisher.publish(msg)