---
sidebar_position: 2
---

# 7.2 Physics Simulation Concepts

To create a valuable Digital Twin, the simulation must be more than just a pretty picture; it must be physically plausible. Gazebo uses high-performance physics engines (like ODE or Bullet) to compute the effects of forces and collisions on your robot model, allowing you to test its dynamics in a realistic way.

### From Kinematics (URDF) to Dynamics (SDF)

In ROS, robot models are often first described using a **URDF (Unified Robot Description Format)** file. URDF is an XML format that defines the robot's kinematic chain: its **links** (the rigid parts) and **joints** (which connect the links and define their motion). URDF is excellent for visualization and kinematic calculations (e.g., in RViz2).

However, URDF cannot describe a robot's full dynamics. For that, Gazebo uses **SDF (Simulation Description Format)**. SDF is a superset of URDF; it includes everything URDF has, plus tags for defining:

-   **Dynamics:** Mass, inertia (how mass is distributed), and friction.
-   **Sensors:** Cameras, LiDAR, IMUs, and their properties.
-   **Physics Properties:** Collision geometry, surface friction, and damping.
-   **Environment:** Lighting, gravity, and entire world models.

*<p align="center">PLACEHOLDER: Diagram comparing URDF and SDF tags, showing SDF adding inertia, collision, sensor, and physics properties to a simple link.</p>*

### Core Concepts for a Dynamic Model

When converting a URDF to an SDF or creating an SDF from scratch, these are the key elements you'll define:

1.  **Collision Geometry:** This is a simplified shape (like a box, cylinder, or sphere) that represents the physical bounds of a link. The physics engine uses these simple shapes for efficient collision detection. It's usually a coarse approximation of the visual mesh.

2.  **Visual Geometry:** This is the detailed 3D mesh (e.g., a `.dae` or `.stl` file) that represents the actual appearance of the link. It's used for rendering only, not for physics calculations.

3.  **Inertia and Mass:** The `<inertial>` tag in SDF is critical. It defines the link's `mass` and its `inertia matrix`. The inertia matrix describes how the link's mass is distributed, which determines how it will react to forces and torques. Getting this right is key to simulating realistic dynamic behavior.

4.  **Joints and Limits:** Joints define the degrees of freedom between links. In SDF, you can specify physical properties for joints, such as `damping` (resistance to motion) and `friction`, as well as effort and velocity limits that the physics engine will enforce.

By carefully defining these properties, you can create a simulation that accurately predicts how your robot will behave under real-world forces.
