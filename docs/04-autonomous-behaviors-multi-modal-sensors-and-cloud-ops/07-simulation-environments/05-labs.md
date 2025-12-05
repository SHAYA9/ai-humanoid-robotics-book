---
sidebar_position: 5
---

# 7.5 Labs: From URDF to a Simulated Robot

These labs will guide you through the process of creating a robot model, adding dynamics to it, and controlling it in the Gazebo simulator.

### Lab 7.1: Create a Kinematic Model (URDF)

**Goal:** Define the kinematic structure of a simple two-link robotic arm using URDF.

1.  **Create a Package:** In your ROS 2 workspace, create a new package named `my_robot_description`.

2.  **Create URDF Directory:** Inside the package, create a directory structure: `urdf/`.

3.  **Write the URDF:** Create a file named `my_arm.urdf` inside the `urdf` directory. Write the XML to define two links (`base_link`, `arm_link`) and a revolute joint (`arm_joint`) connecting them. For the `<visual>` tags, you can use simple `<box>` or `<cylinder>` geometry.

4.  **Create a Launch File:** Create a launch file that starts the `robot_state_publisher` and `joint_state_publisher_gui` nodes. The `robot_state_publisher` will read your URDF and publish the robot's transform (`/tf`) data.

5.  **Visualize in RViz2:** Launch your launch file. Open RViz2 and add a "RobotModel" display. You should see your simple arm. Use the `joint_state_publisher_gui` to move the joint and see the arm articulate in RViz2.

    **Outcome:** A correct kinematic model of a robotic arm, viewable in RViz2.

### Lab 7.2: Add Dynamics (SDF)

**Goal:** Convert the URDF to SDF and add physical properties.

1.  **Convert URDF to SDF:** Gazebo provides a command-line tool to perform a basic conversion.
    ```bash
    gz sdf -p my_arm.urdf > my_arm.sdf
    ```

2.  **Edit the SDF:** Open `my_arm.sdf`. You will see that the file is much more verbose. Your task is to add the following to each link:
    -   An `<inertial>` tag with `mass` and `inertia` matrix values. You can start with placeholder values.
    -   A `<collision>` tag. Copy the geometry from the `<visual>` tag to create a simple collision shape.

    **Outcome:** A dynamically-defined robot model in SDF format.

### Lab 7.3: Spawn in Gazebo

**Goal:** Launch Gazebo and spawn your robot model into the simulation.

1.  **Create a World File:** Create a new directory `worlds` and a simple world file `empty.world` inside it. You can start with a minimal world containing only the ground plane and lighting.

2.  **Update the Launch File:** Create a new launch file, `spawn_robot.launch.py`. This file will:
    -   Start the Gazebo server (`gzserver`) with your `empty.world` file.
    -   Start the Gazebo client (`gzclient`) for visualization.
    -   Use the `spawn_entity.py` script (provided by Gazebo) as a `Node` to spawn your `my_arm.sdf` file into the running simulation.

3.  **Launch:** Run the new launch file.

    **Outcome:** The Gazebo window opens, and you see your robotic arm standing on the ground plane, subject to gravity.

### Module Project: Joint Control

**Goal:** Write a ROS 2 node to control the joint of your simulated arm.

1.  **Examine Gazebo Topics:** With the simulation running, use `ros2 topic list` to see the topics created by Gazebo. You should find a topic for controlling the joint, likely named something like `/model/my_arm/joint/arm_joint/cmd_pos`.

2.  **Write a Publisher Node:** Create a Python node that publishes a `std_msgs/msg/Float64` message to the joint command topic. On a timer, have it publish a sine wave or a simple ramp to move the joint back and forth.

3.  **Create a Final Launch File:** Create a final launch file that starts Gazebo, spawns the robot, and starts your controller node all at once.

    **Outcome:** You can launch your entire simulation with a single command and watch your custom ROS 2 node control the arm in Gazebo. This completes the full workflow from robot description to simulation and control.
