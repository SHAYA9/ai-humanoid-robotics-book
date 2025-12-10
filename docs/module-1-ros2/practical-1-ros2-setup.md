---
sidebar_position: 6
title: 'Practical 1: ROS 2 Setup'
---

# Practical 1: ROS 2 Setup

In this practical, we will set up a ROS 2 workspace, create a package, and build the `talker` and `listener` nodes we wrote in the [`rclpy`](./rclpy-python-agents.md) section.

## Step 1: Install ROS 2 Humble

If you haven't already, follow the official ROS 2 documentation to install ROS 2 Humble Hawksbill on your Ubuntu 22.04 system.

## Step 2: Create a Workspace

A ROS 2 workspace is a directory where you will create and build your ROS 2 packages.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Step 3: Create a Package

Now, let's create a new package for our talker and listener nodes.

```bash
cd src
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy std_msgs
```

This will create a new directory named `my_robot_pkg` with the necessary files for a Python-based ROS 2 package.

## Step 4: Add the Nodes

Create two new files in the `my_robot_pkg/my_robot_pkg` directory:

-   `talker.py`
-   `listener.py`

Copy the code from the [`rclpy`](./rclpy-python-agents.md) section into these files.

## Step 5: Configure the Package

You need to tell the ROS 2 build system about your new nodes. Open the `setup.py` file and add the following `entry_points` to the `console_scripts` section:

```python
entry_points={
    'console_scripts': [
        'talker = my_robot_pkg.talker:main',
        'listener = my_robot_pkg.listener:main',
    ],
},
```

This tells ROS 2 that you want to create two executables, `talker` and `listener`, from the `main` function in the `talker.py` and `listener.py` files, respectively.

## Step 6: Build and Run

Now you can build your package with `colcon`, the ROS 2 build tool.

```bash
cd ~/ros2_ws
colcon build
```

Once the build is complete, you need to source the workspace to make the new executables available.

```bash
source install/setup.bash
```

Now you can run your nodes:

**Terminal 1:**
```bash
ros2 run my_robot_pkg talker
```

**Terminal 2:**
```bash
ros2 run my_robot_pkg listener
```

You should see the "Hello World" messages being published and received. Congratulations, you have successfully created and run your first ROS 2 package!
