---
sidebar_position: 6
title: 'Practical 3: Perception'
---

# Practical 3: Perception with Isaac ROS

In this practical exercise, you'll get hands-on experience with the **Isaac ROS** perception packages. You'll set up a simple perception pipeline in Isaac Sim and visualize the results in RViz.

## Objectives

1.  **Set up a scene in Isaac Sim** with a robot and some objects.
2.  **Publish stereo camera images** from Isaac Sim to ROS 2.
3.  **Use the Isaac ROS stereo_image_proc** package to generate a disparity image and a point cloud.
4.  **Use the Isaac ROS dnn_image_encoder** package to run an object detection model on the camera images.
5.  **Visualize the results** (point cloud, detected object bounding boxes) in RViz.

## Steps

*   **Step 1: The Isaac Sim Scene.** Load a pre-built scene in Isaac Sim or create your own with a robot (e.g., the Carter robot) and a few simple objects.
*   **Step 2: ROS 2 Bridge.** Use Isaac Sim's built-in ROS 2 bridge to publish stereo camera images and camera info topics.
*   **Step 3: The Perception Pipeline.** Create a ROS 2 launch file that starts the Isaac ROS packages for stereo processing and object detection. You'll need to configure the topics and model parameters.
*   **Step 4: Visualization.** Launch RViz and subscribe to the output topics from your perception pipeline. You should be able to see the 3D point cloud of the scene and the bounding boxes around the detected objects.

This exercise will give you a practical understanding of how to build a hardware-accelerated perception pipeline with Isaac ROS.
