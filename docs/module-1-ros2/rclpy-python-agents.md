---
sidebar_position: 4
title: rclpy - Python Agents for ROS 2
---

# `rclpy`: Python Agents for ROS 2

`rclpy` is the official Python client library for ROS 2. It provides a high-level, Pythonic interface to the ROS 2 ecosystem, allowing you to write your own ROS 2 nodes, publishers, subscribers, and more.

## Getting Started with `rclpy`

Before you can start writing `rclpy` nodes, you need to have a ROS 2 workspace set up. If you haven't done so already, please refer to the official ROS 2 installation guide.

### A Minimal `rclpy` Node

Here is the "Hello World" of `rclpy`: a simple node that initializes itself and then shuts down.

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create a node
    node = Node('my_first_node')

    # Keep the node alive until it's shut down
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This simple example demonstrates the basic structure of a `rclpy` program. Let's break it down:

1.  `rclpy.init()`: Initializes the ROS 2 communication.
2.  `Node('my_first_node')`: Creates a new node with the name `my_first_node`.
3.  `rclpy.spin(node)`: Enters a loop that keeps the node running and responsive to events.
4.  `node.destroy_node()` and `rclpy.shutdown()`: Cleans up the node and the ROS 2 communication.

## Publishers and Subscribers

The real power of ROS 2 comes from the communication between nodes. Let's create a simple publisher and subscriber to see this in action.

### Publisher Node

This node will publish a `String` message to the `/chatter` topic every second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node

This node will subscribe to the `/chatter` topic and print any messages it receives.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In the [Practical 1: ROS 2 Setup](./practical-1-ros2-setup.md), you will learn how to create a ROS 2 package, build these nodes, and run them to see them communicate. Next, we'll learn how to represent our robot's structure with [URDF for Humanoids](./urdf-humanoids.md).
