---
id: faq
title: "Frequently Asked Questions (FAQ)"
sidebar_label: "FAQ"
description: "Common questions and answers about Physical AI & Humanoid Robotics course"
tags: [faq, questions, help, troubleshooting]
---

# Frequently Asked Questions (FAQ)

## 🤖 General Course Questions

### Q1: Who is this course for?
**A:** This course is designed for:
- Computer Science/Engineering students interested in robotics
- AI researchers wanting to work with physical systems
- Robotics engineers upgrading to AI-powered systems
- Hobbyists with technical background looking to build advanced projects
- **Prerequisites:** Basic Python programming, understanding of linear algebra, familiarity with Linux

### Q2: What's the difference between Physical AI and regular AI?
**A:** 
| **Physical AI** | **Digital AI** |
|----------------|---------------|
| Operates in physical world | Operates in digital space |
| Understands physics laws | Understands information patterns |
| Deals with sensor noise | Works with clean data |
| Real-time constraints | Can be batch processed |
| Safety-critical | Lower stakes |
| Examples: Self-driving cars, robots | Examples: Chatbots, image generators |

### Q3: How much time should I dedicate weekly?
**A:** 
- **Minimum:** 10-12 hours per week (4 hours lectures/labs, 6-8 hours practice)
- **Recommended:** 15-20 hours per week for deeper understanding
- **Capstone weeks:** 25-30 hours (intensive project work)

## 💻 Technical Setup Questions

### Q4: Do I need to buy all the hardware listed?
**A:** No, you have options:
1. **Cloud-only path:** Use AWS/Azure instances + basic Jetson kit ($700-1000)
2. **Hybrid path:** Use your existing computer + cloud for heavy simulations
3. **Full setup:** Complete workstation + robot ($3000-5000)

**Minimum requirement:** Any computer that can run Ubuntu 22.04 + access to cloud GPU for Module 3-4.

### Q5: Can I use Windows/Mac instead of Ubuntu?
**A:** 
- **Module 1-2:** Possible with WSL2 (Windows) or Virtual Machine
- **Module 3-4:** **Not recommended** - NVIDIA Isaac Sim requires native Linux
- **Best option:** Dual-boot Ubuntu or use a dedicated Linux machine

### Q6: What if I don't have an NVIDIA GPU?
**A:** Three options:
1. **Cloud GPUs:** Use AWS/Azure/Google Cloud (cost: ~$200 for the course)
2. **University resources:** Check if your institution has GPU clusters
3. **Google Colab Pro:** Limited but works for training smaller models
4. **CPU-only:** Possible but very slow for Isaac Sim - not recommended

## 🛠️ ROS 2 Questions

### Q7: ROS 1 vs ROS 2 - which should I learn?
**A:** 
- **This course:** ROS 2 Humble (latest LTS version)
- **Industry trend:** New projects use ROS 2, legacy uses ROS 1
- **Key differences:** 
  - ROS 2 has real-time capabilities
  - ROS 2 supports multiple DDS implementations
  - ROS 2 has improved security
  - ROS 1 is being phased out

### Q8: ROS 2 keeps giving "package not found" errors
**A:** Common solutions:

```bash
# 1. Source your workspace
source ~/autonomous_humanoid_ws/install/setup.bash

# 2. Check if package is built
cd ~/autonomous_humanoid_ws
colcon list -p | grep <package_name>

# 3. Rebuild if needed
colcon build --packages-select <package_name>

# 4. Check dependencies
rosdep install --from-paths src --ignore-src -r -y