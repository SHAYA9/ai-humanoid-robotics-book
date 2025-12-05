---
sidebar_position: 3
---

# 7.3 Synthetic Data Generation (SDG)

One of the most compelling reasons to use a photorealistic simulator like Isaac Sim is for **Synthetic Data Generation (SDG)**. Training modern perception models, especially for object detection and segmentation, requires enormous, hand-labeled datasets. SDG automates this process, allowing you to generate perfectly labeled data at a massive scale.

### The Core Tool: Isaac Replicator

The engine behind SDG in Isaac Sim is **Replicator**. Replicator is a powerful Python framework that allows you to programmatically control and randomize your simulation to create diverse datasets.

A typical Replicator script performs the following steps in a loop:

1.  **Randomize the Scene:** Change various properties of the environment.
2.  **Capture Sensor Data:** Render images, depth maps, etc.
3.  **Generate Labels:** Output the corresponding ground-truth labels.

### Domain Randomization: The Key to Sim-to-Real

If you train a model on synthetic data that looks exactly the same in every image, it will fail in the real world where conditions are variable. The key to creating robust models is **domain randomization**. This involves randomizing every parameter you can think of to expose the model to a wide variety of conditions.

With Replicator, you can easily randomize:

-   **Object Poses:** The position and orientation of objects of interest.
-   **Lighting:** The color, intensity, and position of lights in the scene.
-   **Textures:** The materials and textures of objects and the background.
-   **Camera Position:** The location and angle of the camera capturing the data.

*<p align="center">PLACEHOLDER: Domain Randomization Example Image</p>*
*<p align="center">A grid of images showing the same scene but with different lighting, object positions, and textures, illustrating the concept of domain randomization.</p>*

### Generated Data Types

Replicator can output a wide variety of ground-truth data that is perfectly synchronized with the rendered images:

-   **RGB Images**
-   **Depth Maps**
-   **Semantic Segmentation:** Every pixel is labeled with the class of object it belongs to (e.g., "table", "chair", "robot").
-   **Instance Segmentation:** Every pixel is labeled with the specific instance of the object it belongs to (e.g., "chair_1", "chair_2").
-   **2D and 3D Bounding Boxes:** The precise coordinates of the box enclosing each object.

Because this data is generated automatically, it is free from human labeling errors and can be created orders of magnitude faster and cheaper than manual labeling. This makes SDG an essential tool for modern, AI-driven robotics.
