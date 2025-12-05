---
sidebar_position: 5
---

# 7.5 Visualization and Debugging

A running ROS 2 system is a complex, distributed network of nodes passing messages. Without the right tools, understanding what's happening can be nearly impossible. Fortunately, ROS 2 comes with a powerful suite of command-line and graphical tools for introspection, debugging, and visualization.

### Introspection with `ros2` CLI Tools

The `ros2` command is your primary entry point for interacting with a live ROS 2 system from the terminal. It provides a set of verbs to inspect different parts of the ROS graph.

-   **`ros2 node list`**: Shows the names of all currently running nodes. This is the first command you should run to see if your system is alive.

-   **`ros2 topic list`**: Lists all active topics. Use the `-t` flag (`ros2 topic list -t`) to also see the message type for each topic.

-   **`ros2 topic echo <topic_name>`**: Prints the messages being published on a specific topic to the console. This is incredibly useful for debugging sensor data or node outputs without writing a new subscriber node.
    ```bash
    ros2 topic echo /chatter
    ```

-   **`ros2 topic pub <topic_name> <message_type> '<yaml_data>'`**: Publishes a single message to a topic from the command line. This is great for testing a node in isolation by sending it a specific input.
    ```bash
    ros2 topic pub /chatter std_msgs/msg/String '{data: "Hello from CLI"}' -1
    ```

-   **`ros2 service list`**: Lists all available services.
-   **`ros2 service call <service_name> <service_type> '<yaml_data>'`**: Calls a service and prints the response.

### Visualizing the Node Graph with `rqt_graph`

While the CLI tools are great for inspecting individual parts, `rqt_graph` gives you a high-level, visual overview of the entire system. It draws a diagram of all the nodes and shows how they are connected via topics.

To run it, simply execute:
```bash
rqt_graph
```

*<p align="center">PLACEHOLDER: `rqt_graph` Screenshot</p>*
*<p align="center">A screenshot showing the `rqt_graph` window with nodes represented as ovals and topics as squares, with arrows indicating the flow of data.</p>*

This tool is invaluable for:
-   Verifying that your system is wired up as you expect.
-   Finding disconnected nodes or topics with typos.
-   Understanding the data flow in a complex application you've never seen before.

### Visualizing Robot Data with RViz2

The most powerful visualization tool in the ROS 2 ecosystem is **RViz2**. It's a 3D visualizer that can subscribe to a wide variety of ROS 2 topics and display their data in a configurable 3D scene.

RViz2 is essential for:
-   Visualizing sensor data like camera images, LiDAR scans (`LaserScan`), and point clouds (`PointCloud2`).
-   Displaying the state of a robot model (URDF) by listening to joint states.
-   Showing planned paths, detected obstacles, and coordinate frames (`tf`).
-   Creating interactive markers to send commands to a robot.

*<p align="center">PLACEHOLDER: RViz2 Screenshot</p>*
*<p align="center">A screenshot of the RViz2 interface showing a robot model, a LiDAR scan, and a camera feed in a 3D environment.</p>*

By adding different "Displays" in the left-hand panel, you can customize RViz2 to visualize almost any data in your system, making it the go-to tool for debugging perception, localization, and navigation algorithms.
