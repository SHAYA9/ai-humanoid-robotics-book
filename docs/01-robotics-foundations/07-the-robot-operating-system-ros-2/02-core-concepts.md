---
sidebar_position: 2
---

# 7.2 Core Concepts: Nodes, Topics, and Messages

At the heart of ROS 2 is a communication graph where independent programs exchange information. Understanding the three fundamental components of this graph—Nodes, Topics, and Messages—is the key to mastering ROS 2.

### The ROS 2 Graph

Imagine a group of experts in a room. Each expert is a **Node**. They communicate by writing announcements on a shared whiteboard. Each section of the whiteboard is a **Topic**. The format of the announcement is the **Message**.

-   **Nodes:** A node is the smallest, most fundamental unit of a ROS 2 system. It's an independent program that performs a specific task. For example, you might have one node for a camera driver, another for image processing, and a third for motor control. Each node is a self-contained executable that can be started, stopped, and restarted independently.

-   **Topics:** A topic is a named bus or channel that nodes use to exchange data. Topics are the primary method for asynchronous, many-to-many communication. A node can **publish** data to a topic, and any number of other nodes can **subscribe** to that topic to receive the data. For instance, a camera node would publish images to an `/image_raw` topic, and both an image processing node and a display node could subscribe to it.

-   **Messages:** When a node publishes data to a topic, it does so using a **message**. A message is a simple data structure with a defined type. ROS 2 provides many standard message types (e.g., `String`, `Int32`, `Point`) and you can define your own custom messages. For example, the `/image_raw` topic would use the `sensor_msgs/Image` message type, which is a standardized structure for holding image data.

### Creating Your First `rclpy` Node

Let's see how this looks in practice. In ROS 2, `rclpy` is the official Python client library. Here is a minimal "Hello World" node:

```python
#
#
import rclpy
from rclpy.node import Node

def main(args=None):
    # 1. Initialize the rclpy library
    rclpy.init(args=args)

    # 2. Create a Node
    #    The node is named 'my_first_node'
    node = Node('my_first_node')

    #    (We'll add publishers and subscribers here later)
    print("My first node is alive!")

    # 3. "Spin" the node to keep it running and responsive
    #    to callbacks (like receiving messages).
    #    This will block until the node is shut down (e.g., by Ctrl+C).
    rclpy.spin(node)

    # 4. Cleanly shut down the node and rclpy
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Implementing Publishers and Subscribers

Now, let's make two nodes that actually communicate.

**Publisher Node (`talker.py`):** This node will publish a `String` message to a topic named `/chatter` every second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        # Create a publisher
        # Message type: String
        # Topic name: /chatter
        # Queue size: 10 (how many messages to buffer)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.counter_ = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.counter_}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.counter_ += 1

# ... (main function similar to above, but creating a TalkerNode)
```

**Subscriber Node (`listener.py`):** This node subscribes to `/chatter` and prints any message it receives.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        # Create a subscriber
        # Message type: String
        # Topic name: /chatter
        # Callback function: self.listener_callback
        # Queue size: 10
        self.subscription_ = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: '{msg.data}'")

# ... (main function similar to above, but creating a ListenerNode)
```

When you run both of these nodes, you will see the `talker` publishing messages and the `listener` receiving them. This simple, decoupled communication is the foundation of every ROS 2 application.
