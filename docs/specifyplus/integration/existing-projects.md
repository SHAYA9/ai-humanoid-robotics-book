# Integration: Existing Projects

Integrating an existing robotics project with the SpecifyPlus framework primarily involves ensuring compatibility with the ROS 2 communication layer. This guide provides steps for adapting your existing ROS 1 or ROS 2 projects.

## Case 1: Integrating an Existing ROS 2 Project

This is the most straightforward case. The main task is to align your project's topics, services, and message types with the conventions used in SpecifyPlus.

1.  **Analyze Interfaces:**
    -   Use `ros2 node info <your_node>` to list all the publishers, subscribers, services, and actions for your existing nodes.
    -   Compare these with the standard interfaces used in SpecifyPlus (e.g., `/cmd_vel` for motion, `/joint_states` for robot state).

2.  **Topic/Service Remapping:**
    -   The easiest way to adapt your node is by using remapping rules in your launch file. This allows you to change the names of topics and services without modifying your code.

    **Example Launch File:**
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='my_existing_package',
                executable='my_planner_node',
                name='my_planner',
                remappings=[
                    ('/my_planner/output_path', '/plan'), # Remap output topic
                    ('/input_scan', '/scan')             # Remap input topic
                ]
            )
        ])
    ```

3.  **Message Type Conversion:**
    -   If your project uses custom message or service types, you may need to create a "translator" node. This node subscribes to your custom message topic, converts the data into a standard message type (e.g., a `sensor_msgs/msg/Image`), and republishes it on a new topic for the rest of the SpecifyPlus system to use.

## Case 2: Integrating a ROS 1 Project

To integrate a project written in ROS 1, you must use the `ros1_bridge`. This bridge acts as a bidirectional translator for topics, services, and actions between a ROS 1 graph and a ROS 2 graph.

1.  **Install the Bridge:**
    ```bash
    sudo apt-get install ros-humble-ros1-bridge
    ```

2.  **Run the Bridge:**
    -   First, start your ROS 1 `roscore` and your ROS 1 nodes in one set of terminals.
    -   Then, in a separate terminal, source your ROS 2 environment and run the bridge.
    ```bash
    # Terminal 1: ROS 1
    source /opt/ros/noetic/setup.bash
    roscore

    # Terminal 2: ROS 1
    source /opt/ros/noetic/setup.bash
    rosrun my_ros1_package my_ros1_node

    # Terminal 3: Bridge
    source /opt/ros/humble/setup.bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    ```
    -   The `--bridge-all-topics` flag will automatically bridge all topics it discovers. You can also configure the bridge to only pass specific topics for better performance.

3.  **Verify Communication:**
    -   In a ROS 2 terminal, you should now be able to see the topics published by your ROS 1 node.
    -   For example, if your ROS 1 node publishes on `/camera/image`, you can check it in ROS 2:
    ```bash
    # Terminal 4: ROS 2
    source /opt/ros/humble/setup.bash
    ros2 topic list
    ros2 topic echo /camera/image
    ```

## Case 3: Integrating a Non-ROS Project

If your project is not based on ROS, you will need to create a ROS 2 "wrapper" node.

1.  **Identify Entry Points:** Determine how to get data into and out of your existing application (e.g., via a C++ or Python API, a command-line interface, or a network socket).

2.  **Create a Wrapper Node:**
    -   Create a new ROS 2 package and node.
    -   This node will act as a bridge. It will subscribe to ROS 2 topics, and when it receives a message, it will call the appropriate function in your non-ROS application's API.
    -   Conversely, if your application produces data, the wrapper node will be responsible for polling that data, converting it into a ROS 2 message, and publishing it on a topic.

This approach encapsulates your external project, making it behave like any other native ROS 2 node within the SpecifyPlus ecosystem.
