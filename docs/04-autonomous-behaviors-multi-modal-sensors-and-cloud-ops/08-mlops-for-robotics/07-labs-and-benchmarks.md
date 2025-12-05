---
sidebar_position: 7
---

# 7.7 Labs and Benchmarks

These labs will provide hands-on experience with the core components of the NVIDIA Isaac ecosystem. **Please ensure your system meets the GPU requirements outlined in the previous section before proceeding.**

### Lab 7.1: Isaac Sim Installation & "Hello World"

**Goal:** Install and run Isaac Sim for the first time.

1.  **Install Omniverse Launcher:** Download and install the NVIDIA Omniverse Launcher from the NVIDIA website.
2.  **Install Isaac Sim:** From the "Exchange" tab in the Launcher, find and install the recommended version of Isaac Sim.
3.  **Launch Isaac Sim:** Open the "Library" tab and launch the installed Isaac Sim application.
4.  **Load a Sample Scene:** Once loaded, go to the top menu `File > Open Sample` and choose the `CarterV2_Warehouse_Navigation.usd` scene.
5.  **Run the Simulation:** Press the "Play" button in the main viewport.
6.  **Drive the Robot:** Select the robot in the viewport. In the property panel below, under the "Raw USD Properties" for the `controller` prim, you can use the keyboard arrow keys to drive the Carter robot around the warehouse.

    **Outcome:** A working installation of Isaac Sim and a running, interactive robotics simulation.

### Lab 7.2: Synthetic Data Generation for Object Detection

**Goal:** Use the Replicator framework to generate a small, labeled dataset.

1.  **Open the Replicator Example:** In Isaac Sim, go to `Window > Extensions` and enable the `omni.isaac.replicator` extension. Then, go to `Isaac Utils > Replicator > Code Examples` and open the "Basic Usage" example.
2.  **Understand the Script:** Read through the Python script. It shows how to spawn objects, randomize their materials and positions, and attach writers to output data like RGB images and bounding box labels.
3.  **Run the Script:** Execute the script from the provided UI.
4.  **Inspect the Output:** Check the specified output directory. You will find a series of images and the corresponding label files.

    **Outcome:** A small, synthetically generated dataset, demonstrating the core workflow of Replicator.

### Lab 7.3: Isaac ROS VSLAM

**Goal:** Run the GPU-accelerated VSLAM Gem with simulated data from Isaac Sim.

1.  **Run Isaac ROS Docker:** Follow the Isaac ROS documentation to pull and run the VSLAM Docker container. This container has all the necessary libraries pre-installed.
2.  **Launch the Isaac Sim Scene:** Open the Isaac Sim scene that is pre-configured for VSLAM (e.g., the `carter_v2_stereo_vslam.usd` scene). This scene contains a robot with the required stereo cameras and IMU.
3.  **Launch the VSLAM Node:** Inside the Docker container, use `ros2 launch` to start the Isaac ROS VSLAM launch file.
4.  **Visualize in RViz2:** Open RViz2 and configure it to display the camera feeds, the generated point cloud map, and the robot's odometry. Drive the robot around in Isaac Sim.

    **Outcome:** You will see the VSLAM system building a map and tracking the robot's position in real-time, all accelerated by the GPU.

### Module Project: Warehouse Navigation with Nav2

**Goal:** Integrate the ROS 2 Nav2 stack to perform autonomous navigation in Isaac Sim.

1.  **Launch the Nav2 Scene:** Open the `CarterV2_Warehouse_Navigation.usd` scene in Isaac Sim. This scene is already configured with a LiDAR-equipped Carter robot and a ROS 2 Bridge.
2.  **Launch Nav2:** In a terminal (outside of Isaac Sim), launch the Nav2 stack, using the configuration files provided for the Carter robot.
3.  **Localize and Navigate:**
    -   In RViz2, provide an initial pose estimate for the robot so AMCL can localize it.
    -   Use the "Nav2 Goal" button in RViz2 to set a navigation goal somewhere in the warehouse.

    **Outcome:** The simulated Carter robot will autonomously navigate through the warehouse to the goal, avoiding obstacles, all orchestrated by the Nav2 stack running outside the simulator. This demonstrates the complete sim-to-real workflow.

### Benchmarks (Optional)

-   **Task:** Run the Isaac ROS VSLAM and a popular CPU-based VSLAM (like ORB-SLAM3) on the same dataset or simulation.
-   **Metric:** Compare the processing speed (frames per second) and the CPU vs. GPU utilization for each.
-   **Expected Result:** The Isaac ROS VSLAM should show significantly higher throughput and lower CPU usage, demonstrating the benefit of GPU acceleration.
