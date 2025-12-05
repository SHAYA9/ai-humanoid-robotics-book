---
sidebar_position: 4
---

# 7.4 Managing Complexity: Launch Files and Parameters

As your robotics application grows from two nodes to ten, twenty, or even a hundred, starting and configuring each one manually becomes impossible. This is where ROS 2's launch system comes in. **Launch files** are powerful scripts that allow you to start, configure, and connect a complex system of nodes with a single command.

### Introduction to Launch Files

In ROS 2, launch files are typically written in Python. They provide a programmatic way to describe your system's startup configuration. A single launch file can:

-   Start multiple nodes.
-   Set parameters for each node.
-   Remap topic names to connect nodes in different ways.
-   Group nodes into namespaces to avoid conflicts.
-   Conditionally launch nodes based on arguments.

This approach allows you to define your entire application as a single, version-controllable configuration, making it repeatable and easy to share.

#### Example: Launching the Talker and Listener

Remember the `talker` and `listener` nodes from section 7.2? Instead of running them in two separate terminals, we can create a single launch file to start both.

**`talker_listener.launch.py`**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This function is the entry point for the launch file

    # Create a LaunchDescription object to hold our actions
    ld = LaunchDescription()

    # Define the talker node
    talker_node = Node(
        package='my_robot_pkg',      # The name of your ROS 2 package
        executable='talker',         # The name of the executable (from setup.py)
        name='my_talker'             # An optional custom name for the node
    )

    # Define the listener node
    listener_node = Node(
        package='my_robot_pkg',
        executable='listener',
        name='my_listener'
    )

    # Add the nodes to the LaunchDescription
    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld
```
Now, you can run `ros2 launch my_robot_pkg talker_listener.launch.py` to start both nodes at once.

*<p align="center">PLACEHOLDER: Launch File Workflow Diagram</p>*
*<p align="center">A diagram showing `ros2 launch` executing the Python script, which generates a description that the launch system uses to start and configure the talker and listener nodes.</p>*

### Using Parameters to Configure Nodes

Hardcoding values inside your nodes is inflexible. What if you want to change the frequency of your `talker` node without editing the code? **Parameters** are the solution. They are a built-in ROS 2 feature that allows you to store and modify configuration values for a node while it's running.

#### Declaring and Using a Parameter in a Node

First, you need to declare a parameter in your node to make it available. Let's modify our `talker` node to have a configurable `publish_frequency`.

**`talker_with_params.py`**
```python
# ... imports
class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')

        # 1. Declare the parameter
        #    Name: 'publish_frequency'
        #    Default value: 1.0 (Hz)
        self.declare_parameter('publish_frequency', 1.0)

        # 2. Get the parameter's value
        publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        timer_period = 1.0 / publish_frequency # Convert frequency to period

        # Create the publisher and timer as before, but using the parameter
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        # ... rest of the class
```

#### Setting a Parameter from a Launch File

Now, we can set this parameter's value directly from our launch file.

**`talker_configured.launch.py`**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='talker',
            name='my_configured_talker',
            parameters=[
                # Set the 'publish_frequency' parameter to 2.0 Hz
                {'publish_frequency': 2.0}
            ]
        )
    ])
```
When you run this launch file, the `talker` node will start and publish messages at 2 Hz instead of the default 1 Hz. This allows you to create highly configurable and reusable nodes.

*<p align="center">PLACEHOLDER: Node Parameter Configuration Diagram</p>*
*<p align="center">A diagram showing the launch file passing a parameter value to the ROS 2 parameter server, which the `talker` node then reads during its initialization.</p>*

In **Lab 7.4: Putting It All Together with a Launch File**, you will apply these concepts to create a launch file that starts and configures a complete system, demonstrating the power of the ROS 2 launch system for managing complex applications.
