# Module 2: Simulation Tools

This module covers the use of simulation tools to create a "Digital Twin" of the robot and its environment. Simulation is a critical part of the SpecifyPlus workflow, enabling rapid development, testing, and data generation.

## Gazebo Physics Simulation

Gazebo is the primary tool used for simulating the physics and dynamics of the humanoid robot.

-   **World Files:** The environment in Gazebo is defined in a `.world` file, which includes models for the ground, walls, objects, and lighting.
-   **Robot Spawning:** The robot's URDF model is spawned into the Gazebo world. Gazebo parses the URDF to create the physical properties of the robot, including mass, inertia, and collision shapes.
-   **ROS 2 Integration:** Gazebo is tightly integrated with ROS 2. It subscribes to ROS 2 topics for joint commands and publishes sensor data back to the ROS 2 graph. The `ros_gz_bridge` package is used to translate messages between Gazebo's transport layer and ROS 2.

**Example: Spawning a Robot in Gazebo**

```bash
ros2 launch my_robot_gazebo spawn_robot.launch.py
```

## Unity High-Fidelity Rendering

For tasks that require photorealistic graphics, such as training a vision-based AI model or creating marketing materials, SpecifyPlus can interface with the Unity engine.

-   **ROS-Unity Bridge:** A set of packages, often referred to as `ROS-TCP-Connector`, enables communication between the ROS 2 network and a Unity scene.
-   **State Synchronization:** The state of the robot in the ROS 2 world (e.g., joint positions) is sent to Unity, which updates the visual representation of the robot in real-time.
-   **Synthetic Data Generation:** Unity can be used to generate large, perfectly labeled synthetic datasets for training machine learning models. For example, it can render camera images while also providing pixel-perfect semantic segmentation masks or depth images.

## Sensor Simulation

A key advantage of simulation is the ability to model a wide variety of sensors without needing physical hardware.

### LiDAR Simulation

Gazebo can simulate a 2D or 3D LiDAR sensor. It performs raycasting in the simulated world and publishes the resulting point cloud as a `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2` message on a ROS 2 topic.

### Depth Camera Simulation

Depth cameras (like an Intel RealSense) are simulated by rendering a depth map from a virtual camera's perspective. This is published as a `sensor_msgs/msg/Image` with a depth encoding (e.g., `16UC1`).

### IMU (Inertial Measurement Unit) Simulation

An IMU sensor is simulated by accessing the ground truth state of the robot's link from the physics engine. Gazebo can then calculate the linear acceleration and angular velocity and publish it as a `sensor_msgs/msg/Imu` message, often with added noise to better mimic a real-world sensor.

**Diagram: Data Flow in Simulation**

```mermaid
graph TD
    subgraph ROS 2 Network
        A[Control Node] --> B[/joint_commands];
        C[/sensor_data] --> D[Perception Node];
    end

    subgraph "Simulation (Gazebo/Unity)"
        E[Robot Model];
        F[Simulated Sensors];
        G[Physics Engine];
    end

    B --> G;
    G --> E;
    E --> F;
    F --> C;
```
