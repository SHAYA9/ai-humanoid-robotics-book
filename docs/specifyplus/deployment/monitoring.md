# Deployment: Monitoring

Monitoring the health and performance of a complex, distributed system like SpecifyPlus is crucial for debugging, performance tuning, and ensuring reliability. The ROS 2 ecosystem provides a rich set of tools for introspection and monitoring.

## Command-Line Tools

These are the first-line tools for inspecting the state of a running system.

-   `ros2 node list`: Lists all running nodes.
-   `ros2 topic list`: Lists all active topics.
-   `ros2 topic echo <topic_name>`: Prints the messages being published on a topic.
-   `ros2 topic hz <topic_name>`: Measures the publishing rate of a topic.
-   `ros2 service list`: Lists all active services.
-   `ros2 action list`: Lists all active action servers.
-   `ros2 param list`: Lists all nodes that have parameters.
-   `ros2 param get <node_name> <param_name>`: Gets the value of a parameter.

## Graphical Tools

### RQT (ROS Qt)

RQT is a framework for building GUI tools for ROS. Several useful plugins come pre-installed.

-   `rqt_graph`: Visualizes the ROS 2 graph, showing nodes and the topics that connect them. This is invaluable for understanding the architecture of your running system.
-   `rqt_plot`: Plots numeric values from topics over time. Useful for tuning controllers or monitoring sensor values.
-   `rqt_console`: Displays logging messages from all nodes and allows you to filter by severity level.
-   `rqt_image_view`: Displays video streams from `sensor_msgs/msg/Image` topics.

To run RQT with a specific plugin:

```bash
rqt --standalone rqt_graph
```

### RViz2

RViz is a powerful 3D visualization tool. It subscribes to ROS 2 topics and displays their data in a 3D scene.

Common visualizations include:

-   **Robot Model:** Displays the robot's URDF model, positioned according to the `/tf` data.
-   **LiDAR Scans:** Shows `LaserScan` data as points in the 3D world.
-   **Camera Feeds:** Can display camera images overlaid in the 3D scene.
-   **Maps:** Displays occupancy grids from a mapping or SLAM node.
-   **Paths:** Visualizes the planned path from the Nav2 stack.

## Logging

ROS 2 has a built-in logging framework. Nodes can log messages at different severity levels (DEBUG, INFO, WARN, ERROR, FATAL).

-   **Viewing Logs:** By default, logs are printed to the console and can be viewed with `rqt_console`.
-   **Changing Log Level:** You can change a node's logging verbosity at runtime, which is useful for debugging a specific problem.
    ```bash
    ros2 run my_package my_node --ros-args --log-level debug
    # or for a running node
    ros2 service call /my_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels "{levels: [{name: 'rcl.logging_rosout', level: 10}]}"
    ```

## System-Level Monitoring

For production deployments, you should also monitor the underlying system resources.

-   **CPU & Memory:** Use standard Linux tools like `top`, `htop`, and `free`.
-   **GPU:** Use `nvidia-smi` to monitor GPU utilization, memory, and temperature, which is critical for the Isaac components.
-   **Network:** Use tools like `nethogs` to monitor bandwidth usage, especially in a hybrid cloud deployment.

Integrating these tools into a comprehensive dashboard (e.g., using Grafana and Prometheus) is a best practice for long-term monitoring and alerting.
