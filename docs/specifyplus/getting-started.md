# Getting Started with SpecifyPlus

This guide will walk you through setting up a minimal SpecifyPlus environment and running a basic simulation.

## Prerequisites

-   Ubuntu 22.04
-   ROS 2 Humble Hawksbill installed
-   Gazebo simulator installed
-   Basic knowledge of ROS 2 concepts (nodes, topics)

## Step 1: Create a ROS 2 Workspace

First, create a new ROS 2 workspace to house your project.

```bash
mkdir -p ~/specifyplus_ws/src
cd ~/specifyplus_ws
```

## Step 2: Set up a Simple rclpy Agent

Create a simple Python package for a ROS 2 node.

```bash
cd src
ros2 pkg create --build-type ament_python my_robot_agent --dependencies rclpy
```

Now, create a simple publisher node inside `my_robot_agent/my_robot_agent/`. Name the file `agent_node.py`.

```python
# my_robot_agent/my_robot_agent/agent_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyAgent(Node):
    def __init__(self):
        super().__init__('my_agent_node')
        self.publisher_ = self.create_publisher(String, 'agent_chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from SpecifyPlus Agent: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    agent = MyAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make sure to add the entry point to your `setup.py` file:

```python
# setup.py
...
    entry_points={
        'console_scripts': [
            'agent_node = my_robot_agent.agent_node:main',
        ],
    },
...
```

## Step 3: Build and Run the Agent

Build your workspace and source the setup file.

```bash
cd ~/specifyplus_ws
colcon build
source install/setup.bash
```

Now, run your agent node:

```bash
ros2 run my_robot_agent agent_node
```

In a separate terminal, you can listen to the topic:

```bash
source ~/specifyplus_ws/install/setup.bash
ros2 topic echo /agent_chatter
```

You have now created and run your first SpecifyPlus component! This simple agent forms the basis for more complex behaviors within the system.
