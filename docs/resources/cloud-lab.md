---
id: cloud-lab
title: "Cloud Lab Setup: The Virtual Robotics Lab"
sidebar_label: "Cloud Lab"
description: "Complete guide to setting up your cloud-based robotics development environment"
tags: [cloud, aws, azure, gpu, remote, simulation]
---

# Cloud Lab Setup Guide

## ☁️ Why Cloud for Robotics?

Developing Physical AI applications requires significant computational resources that most personal computers lack. A cloud lab provides:

| Advantage | Description |
|-----------|-------------|
| **Cost Efficiency** | Pay only for what you use vs. buying expensive hardware |
| **Scalability** | Scale GPU power up/down based on needs |
| **Accessibility** | Access from anywhere, any device |
| **Collaboration** | Share environments with team members |
| **Maintenance** | No hardware maintenance, automatic updates |

## 📊 Cloud Provider Comparison

| Provider | Best Instance | GPU | Monthly Cost* | Pros | Cons |
|----------|---------------|-----|---------------|------|------|
| **AWS** | g5.2xlarge | A10G 24GB | $250 | Mature ecosystem, spot instances | Complex pricing |
| **Azure** | NCasT4_v3 | T4 16GB | $220 | Enterprise integration | Limited GPU options |
| **GCP** | n1-standard-8 + T4 | T4 16GB | $230 | Simple pricing | Higher egress costs |
| **Lambda Labs** | GPU.2x | A100 40GB | $300 | Direct GPU access | Smaller ecosystem |

*Based on on-demand pricing, actual costs can be 60-70% lower with spot/preemptible instances

## 🚀 Option A: AWS Setup (Recommended)

### Step 1: Launch Your Instance

**1.1 Login to AWS Console** and navigate to EC2 → Launch Instance

**1.2 Configure Instance:**
- **Name:** `physical-ai-lab`
- **AMI:** `Ubuntu Server 22.04 LTS (HVM), SSD Volume Type`
- **Instance Type:** `g5.2xlarge` (8 vCPU, 32GB RAM, A10G GPU)
- **Key Pair:** Create new key pair `physical-ai-key`

**1.3 Configure Storage:**
- Root volume: 100GB GP3 SSD
- Additional volume: 500GB GP3 SSD for datasets

**1.4 Security Group (Open ports):**
```yaml
- SSH: 22 (Your IP only)
- VNC: 5901 (Your IP only)
- Jupyter: 8888 (Your IP only)
- ROS 2: 11311 (Your IP only)