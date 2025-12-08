# Integration: Customization

The SpecifyPlus framework is designed to be modular and extensible. This guide covers the most common ways to customize the system for your specific robot or application.

## 1. Creating a Custom Robot Description (URDF)

To use your own robot, you need to create a URDF file for it.

1.  **Create a `_description` Package:**
    -   It is standard practice to put all of your robot's description files in a ROS 2 package named `my_robot_description`.
    -   `ros2 pkg create --build-type ament_python my_robot_description`

2.  **Write the URDF:**
    -   Inside this package, create a `urdf/` directory.
    -   Write your URDF file, defining all the links and joints of your robot.
    -   You will also need 3D mesh files (e.g., in `.stl` or `.dae` format) for the visual representation of each link. Place these in a `meshes/` directory.

3.  **Create a Launch File:**
    -   In your description package, create a `launch/` directory.
    -   Create a launch file that starts the `robot_state_publisher` node. This node reads your URDF file, subscribes to the `/joint_states` topic, and broadcasts the 3D pose of all robot links to the `/tf` topic.

**Example `robot_state_publisher.launch.py`:**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'my_robot.urdf'
    )
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        )
    ])
```

## 2. Developing Custom `rclpy` Agents

The core logic of your robot is implemented in custom ROS 2 nodes (agents).

1.  **Create a `_control` Package:**
    -   Create a new package for your custom logic, e.g., `my_robot_control`.

2.  **Write a Node:**
    -   Write a Python script for your node. This node might:
        -   Subscribe to sensor data.
        -   Implement a control algorithm (e.g., PID controller for a joint).
        -   Publish motor commands.
        -   Provide a service for a high-level action.

3.  **Add Entry Point:**
    -   Add your script as an entry point in `setup.py` so you can run it with `ros2 run`.

## 3. Building a Custom Gazebo Environment

You can create a custom simulation environment for your robot to interact with.

1.  **Create a `_gazebo` Package:**
    -   Create a new package, e.g., `my_robot_gazebo`.

2.  **Write a World File:**
    -   Create a `worlds/` directory.
    -   Write a `.world` file in SDF format. This XML file can define everything from the ground plane to buildings, furniture, and lighting.

3.  **Create Models:**
    -   For complex objects in your world, you can create separate models in SDF or URDF format and place them in a `models/` directory.

4.  **Create a Launch File:**
    -   Create a launch file that starts the Gazebo simulator and loads your custom world. It should also include the launch file from your description package to spawn your robot into the world.

**Example `start_simulation.launch.py`:**
```python
# ... imports ...

def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('my_robot_gazebo'),
        'worlds',
        'my_custom_world.world'
    )
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('my_robot_description'),
                         'launch',
                         'robot_state_publisher.launch.py')
        ])
    )

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),
        # Spawn the robot
        spawn_robot_launch
    ])
```
