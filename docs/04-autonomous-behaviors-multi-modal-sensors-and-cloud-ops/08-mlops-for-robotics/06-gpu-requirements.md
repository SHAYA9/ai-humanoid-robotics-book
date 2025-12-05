---
sidebar_position: 6
---

# 7.6 GPU Requirements

The NVIDIA Isaac platform is incredibly powerful, but that power comes with specific hardware requirements. Unlike other ROS 2 tools that are primarily CPU-bound, Isaac Sim and Isaac ROS are fundamentally **GPU-bound**. The performance of your simulation and AI workloads will be directly tied to the capabilities of your NVIDIA graphics card.

This section provides a clear guide to the hardware you will need to follow the labs in this chapter.

### Minimum Requirements

This is the baseline hardware needed to run Isaac Sim and the associated labs. Performance may be limited, and you may need to use lower simulation settings.

-   **Operating System:** Ubuntu 22.04 LTS
-   **NVIDIA Driver:** A recent version (e.g., 525 or newer).
-   **GPU:** An NVIDIA RTX 20-series or Quadro RTX series GPU.
    -   **Examples:** GeForce RTX 2070, Quadro RTX 4000.
-   **VRAM:** **8 GB** of GPU memory is the absolute minimum. Simulations with complex scenes or high-resolution sensors may exceed this.

### Recommended Requirements

For a smooth experience, better performance, and the ability to work with more complex scenes and AI models, the following hardware is recommended.

-   **Operating System:** Ubuntu 22.04 LTS
-   **NVIDIA Driver:** The latest recommended version.
-   **GPU:** An NVIDIA RTX 30-series / 40-series or Ada Lovelace generation GPU.
    -   **Examples:** GeForce RTX 3080, GeForce RTX 4070, NVIDIA RTX A5000.
-   **VRAM:** **16 GB** or more of GPU memory. This will allow for larger synthetic data generation jobs and more complex sensor simulation.

### Why is the GPU so Important?

-   **Ray Tracing (Isaac Sim):** Photorealistic rendering is performed by dedicated RT Cores on NVIDIA RTX GPUs. The more powerful the GPU, the faster and more complex the scenes you can render.
-   **Physics Simulation (Isaac Sim):** PhysX 5 can offload a significant portion of the physics calculations to the GPU's CUDA cores, freeing up the CPU.
-   **AI and Perception (Isaac ROS):** The Isaac ROS Gems, like VSLAM and object detection, are specifically designed to run on the GPU's Tensor Cores for maximum AI inference performance.

Attempting to run Isaac Sim on unsupported hardware (non-NVIDIA or older NVIDIA GPUs) will not work. It is essential to ensure your system meets at least the minimum requirements before proceeding with this module.
