---
id: hardware-setup
title: "Hardware Setup Guide: Building Your Physical AI Lab"
sidebar_label: "Hardware Setup"
description: "Complete hardware guide for Physical AI & Humanoid Robotics - From budget to premium setups"
tags: [hardware, setup, guide, robotics, nvidia, jetson]
---

# Hardware Setup Guide

## 🏗️ Overview: Three Lab Tiers

Choose the setup that matches your budget and learning goals:

| Tier | Budget | Best For | Components |
|------|--------|----------|------------|
| **Tier 1: Cloud-Based** | $500-1000 | Students, Remote Learners | Cloud GPU + Basic Jetson |
| **Tier 2: On-Premise Standard** | $3000-5000 | University Labs, Researchers | RTX Workstation + Jetson + Robot |
| **Tier 3: Premium Sim-to-Real** | $10,000+ | Research Labs, Companies | Multi-GPU + Advanced Humanoid |

## 💻 Tier 1: Cloud-Based Setup (Budget: ~$700)

### 1.1 Cloud GPU Workstation

**Recommended: AWS EC2 g5.2xlarge**
```yaml
Instance: g5.2xlarge
GPU: NVIDIA A10G (24GB VRAM)
vCPU: 8
RAM: 32GB
Storage: 100GB SSD
OS: Ubuntu 22.04 LTS
Monthly Cost: ~$250 (on-demand)
Estimated Project Cost: $205 (using spot instances)