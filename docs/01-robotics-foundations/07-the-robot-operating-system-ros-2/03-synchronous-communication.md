---
sidebar_position: 3
---

# 7.3 Synchronous Communication: Services and Actions

While topics are perfect for continuous data streams, sometimes you need a more direct, synchronous form of communication. For this, ROS 2 provides two powerful mechanisms: **Services** for quick request/response interactions, and **Actions** for long-running tasks that require feedback.

### Services for Request/Response

A **Service** is a synchronous, one-to-one communication pattern. It works just like a function call, but between two different nodes.

-   A **Service Server** node advertises a service with a specific name and type. It waits for a request, performs a computation, and sends back a single response.
-   A **Service Client** node can call the service by sending a request. It then blocks (waits) until it receives the response from the server.

This is ideal for tasks that are quick and have a clear beginning and end, such as:
-   Querying the state of a sensor.
-   Triggering a specific, short-lived behavior (e.g., "take a picture").
-   Requesting a calculation (e.g., "calculate the inverse kinematics for this arm position").

#### Example: An "Add Two Ints" Service

Let's imagine a service that adds two integers. The service definition file (`.srv`) would look like this:

**`AddTwoInts.srv`**
```
int64 a
int64 b
---
int64 sum
```
The `---` separates the request part from the response part.

**Service Server (`add_two_ints_server.py`):**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Using a standard example service

class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        return response
```

**Service Client (`add_two_ints_client.py`):**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        # We can optionally add a callback to run when the future is complete
        # rclpy.spin_until_future_complete(self, self.future)
        return self.future
```
When the client calls `send_request`, the server performs the addition and returns the sum.

### Actions for Long-Running, Feedback-Driven Tasks

What if a task takes a long time to complete, like navigating to a goal or assembling a part? A service isn't a good fit because the client would be blocked for the entire duration. This is where **Actions** come in.

An Action is designed for long-running, asynchronous tasks that can be preempted and provide continuous feedback.

The Action communication involves three parts:
1.  **Goal:** The client sends a goal to the Action Server (e.g., "navigate to position X, Y").
2.  **Feedback:** While the server is working on the goal, it sends a stream of feedback messages to the client (e.g., "current distance to goal is Z meters").
3.  **Result:** When the task is complete, the server sends a final result (e.g., "succeeded" or "failed").

This model is perfect for complex behaviors like:
-   Robot navigation.
-   Executing a multi-step manipulation task.
-   Processing a large dataset.

The client can also send a cancellation request at any time to stop the task.

### Creating Custom Definitions

Just like with messages, you can define your own services and actions.

-   **Service Definitions (`.srv`):** A plain text file with a request part and a response part, separated by `---`.
-   **Action Definitions (`.action`):** A plain text file with three parts: the goal, the result, and the feedback, each separated by `---`.

**`Fibonacci.action`**
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

These definition files are placed in your ROS 2 package and processed by the build system, making them available to your Python and C++ nodes.
