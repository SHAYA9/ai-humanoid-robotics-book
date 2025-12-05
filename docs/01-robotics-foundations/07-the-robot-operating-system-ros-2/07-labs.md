---
sidebar_position: 7
---

# 7.7 Labs

These labs will guide you through the practical steps of creating, building, and running a multi-node ROS 2 application. Following these exercises will solidify the core concepts discussed in this chapter.

### Lab 7.1: Installation and Workspace Setup

**Goal:** Install ROS 2 Humble and create a `colcon` workspace.

1.  **Install ROS 2:** Follow the official ROS 2 Humble Hawksbill installation guide for Ubuntu 22.04. You can find it at [docs.ros.org](https://docs.ros.org/en/humble/Installation.html). Choose the "ROS 2 Desktop" installation.

2.  **Source ROS 2:** After installation, source the main ROS 2 setup file in your terminal. It's good practice to add this to your `.bashrc` file so it's sourced automatically in new terminals.
    ```bash
    source /opt/ros/humble/setup.bash
    ```

3.  **Create a Workspace:** Create a directory for your ROS 2 projects.
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

4.  **Build the Empty Workspace:** Run `colcon build` for the first time. This will create the `install`, `build`, and `log` directories.
    ```bash
    colcon build
    ```

5.  **Source Your Workspace:** Source the new setup file generated in your workspace.
    ```bash
    source install/setup.bash
    ```
    **Outcome:** You now have a functional ROS 2 installation and a ready-to-use workspace.

### Lab 7.2: Simple Publisher and Subscriber

**Goal:** Create a ROS 2 package with two nodes that communicate over a topic.

1.  **Create a Package:** Navigate to your workspace's `src` directory and create a new package.
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy std_msgs
    ```

2.  **Write the Publisher Node:** Inside the `my_robot_pkg/my_robot_pkg` directory, create a file named `talker_node.py` with the publisher code from section 7.2.

3.  **Write the Subscriber Node:** In the same directory, create a file named `listener_node.py` with the subscriber code from section 7.2.

4.  **Edit `setup.py`:** Open `setup.py` and add the entry points for your two nodes in the `console_scripts` section.
    ```python
    'console_scripts': [
        'talker = my_robot_pkg.talker_node:main',
        'listener = my_robot_pkg.listener_node:main',
    ],
    ```

5.  **Build and Run:**
    -   Navigate to the root of your workspace (`~/ros2_ws`).
    -   Build the package: `colcon build`
    -   Source the workspace: `source install/setup.bash`
    -   Open two terminals. In the first, run the talker: `ros2 run my_robot_pkg talker`
    -   In the second, run the listener: `ros2 run my_robot_pkg listener`

    **Outcome:** You should see the talker publishing messages and the listener receiving them.

### Lab 7.3: Service for a Simple Calculator

**Goal:** Implement a client/server pair using a ROS 2 Service.

1.  **Write the Server Node:** In your `my_robot_pkg`, create a file `add_two_ints_server.py` with the server code from section 7.3.
2.  **Write the Client Node:** Create `add_two_ints_client.py` with the client code from section 7.3.
3.  **Update `setup.py`:** Add the new entry points.
    ```python
    'console_scripts': [
        # ... talker and listener
        'add_server = my_robot_pkg.add_two_ints_server:main',
        'add_client = my_robot_pkg.add_two_ints_client:main',
    ],
    ```
4.  **Build and Run:** Rebuild your workspace with `colcon build`. In one terminal, run the server: `ros2 run my_robot_pkg add_server`. In another, run the client with arguments: `ros2 run my_robot_pkg add_client 5 10`.

    **Outcome:** The client should send the numbers to the server, and the server should log the request and the computed sum.

### Lab 7.4: Putting It All Together with a Launch File

**Goal:** Create a launch file to start multiple nodes at once.

1.  **Create a Launch Directory:** Inside your package's root (`my_robot_pkg`), create a directory named `launch`.
    ```bash
    mkdir launch
    ```
2.  **Create the Launch File:** Inside the `launch` directory, create `talker_listener.launch.py` with the launch file code from section 7.4.

3.  **Update `setup.py`:** You need to tell the build system to install the launch file. Add the following to your `data_files` list in `setup.py`.
    ```python
    import os
    from glob import glob
    # ...
    data_files=[
        # ... other data files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    ```
4.  **Build and Launch:** Rebuild the workspace. Now, you can start both nodes with a single command:
    ```bash
    ros2 launch my_robot_pkg talker_listener.launch.py
    ```
    **Outcome:** Both the talker and listener nodes start and begin communicating in a single terminal.

### Module Project: Simulated TurtleBot Control

**Goal:** Use the concepts learned to control a simulated robot.

1.  **Install `turtlesim`:**
    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-turtlesim
    ```
2.  **Start `turtlesim`:**
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    A window with a turtle in the middle should appear.

3.  **Explore the Turtle:**
    -   Use `ros2 topic list` to see the topics `turtlesim` provides. You should see `/turtle1/cmd_vel` for commanding motion and `/turtle1/pose` for getting its position.
    -   Use `ros2 topic pub` to send a single `geometry_msgs/msg/Twist` message to `/turtle1/cmd_vel` to make the turtle move.
    ```bash
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
    ```
4.  **Write a Control Node:**
    -   Create a new node in your package, `turtle_controller.py`.
    -   This node should be a publisher that sends `Twist` messages to `/turtle1/cmd_vel` on a timer to make the turtle draw a shape (e.g., a circle or a square).
    -   Add the entry point to `setup.py` and rebuild.

5.  **Run Your Controller:** Start `turtlesim_node` in one terminal and your `turtle_controller` in another.

    **Outcome:** You have created an integrated ROS 2 application that controls a simulated robot, demonstrating your mastery of the core concepts of nodes, topics, and packages.
