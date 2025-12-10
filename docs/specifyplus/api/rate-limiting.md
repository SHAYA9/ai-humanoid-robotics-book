# API Rate Limiting

In the context of the SpecifyPlus system and ROS 2, "rate limiting" and traffic shaping are managed through two primary mechanisms: the publishing rate of nodes and the Quality of Service (QoS) policies.

## Publishing Rate

The most straightforward way to limit the rate of data on a topic is to control the frequency at which the publisher node publishes messages.

In an `rclpy` agent, this is typically done using a `Timer`. The timer's period directly controls the publication rate.

**Example: A Camera Node Publishing at 10 Hz**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

        # Set a timer to publish at 10 Hz (1 / 0.1s)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

        # self.camera = ... initialize camera hardware

    def capture_and_publish(self):
        # image_msg = self.camera.capture() ... get image from hardware
        # self.publisher_.publish(image_msg)
        self.get_logger().info('Publishing image frame')

# ... main function ...
```

By adjusting the `timer_period`, you can effectively "rate limit" the `/camera/image_raw` topic.

## Quality of Service (QoS) Policies

QoS policies offer a much more sophisticated way to control how data is handled by the underlying DDS communication layer. QoS settings can be configured per-publisher and per-subscriber.

Key QoS settings relevant to rate management:

### Reliability

-   **`RELIABLE`:** Guarantees delivery. If a message is dropped, DDS will try to resend it. This can cause delays if the network is lossy.
-   **`BEST_EFFORT`:** Does not guarantee delivery. This is suitable for high-frequency sensor data where it's better to drop a message than to delay a new one.

### History & Depth

-   **`KEEP_LAST`:** Only stores up to a certain number of recent messages in the queue.
-   **`DEPTH`:** The number of messages to store when using `KEEP_LAST`. If a subscriber is slow, it will only receive the most recent messages up to the depth, and older ones will be dropped. This prevents a slow node from falling behind and being flooded with stale data.

### Durability

-   **`VOLATILE`:** Subscribers will only receive messages that are published after they have connected.
-   **`TRANSIENT_LOCAL`:** A publisher will save the last N messages (where N is the history depth) and send them to any new subscribers that connect. This is useful for topics that publish configuration or state that is needed by late-joining nodes.

### Lifespan

-   **`LIFESPAN`:** Sets an expiration time on messages. If a message is not delivered within its lifespan, it is dropped. This is critical for ensuring that control systems do not act on old, irrelevant data.

**Example: Setting QoS for a LiDAR Scanner**

A LiDAR scanner produces data at a high rate, and the most recent scan is always the most important.

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Define a QoS profile for a sensor
sensor_qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1  # Only care about the absolute latest scan
)

# In the node's __init__ method:
self.subscription = self.create_subscription(
    LaserScan,
    'scan',
    self.listener_callback,
    qos_profile=sensor_qos_profile)
```

By using these QoS policies, you can fine-tune the data delivery behavior of your SpecifyPlus API endpoints to match the specific requirements of your application, preventing network congestion and ensuring real-time performance.
