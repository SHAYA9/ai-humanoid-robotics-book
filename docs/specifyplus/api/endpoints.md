# API Endpoints

While SpecifyPlus does not have a traditional REST or GraphQL API, its functionality is exposed through the ROS 2 graph. This document maps ROS 2 concepts to familiar API terminology. ROS 2 Topics, Services, and Actions serve as the "endpoints" of the system.

## Data Stream Endpoints (ROS 2 Topics)

Topics are endpoints that provide a continuous stream of data. You "subscribe" to a topic to receive data, similar to subscribing to a WebSocket stream.

### `GET /joint_states`

Provides a continuous stream of the robot's joint states.

-   **Topic Name:** `/joint_states`
-   **Message Type:** `sensor_msgs/msg/JointState`
-   **Description:** Publishes the current position, velocity, and effort for all joints of the humanoid robot.
-   **Usage Example (CLI):**
    ```bash
    # Corresponds to a GET request to this endpoint
    ros2 topic echo /joint_states
    ```

### `GET /camera/image_raw`

Provides a video stream from the robot's main camera.

-   **Topic Name:** `/camera/image_raw`
-   **Message Type:** `sensor_msgs/msg/Image`
-   **Description:** Streams raw, uncompressed image frames.
-   **Usage Example (CLI):**
    ```bash
    # This will display the video stream in a window
    ros2 run image_tools showimage --ros-args -p image_topic:=/camera/image_raw
    ```

### `POST /cmd_vel`

Accepts velocity commands to move the robot's base.

-   **Topic Name:** `/cmd_vel`
-   **Message Type:** `geometry_msgs/msg/Twist`
-   **Description:** Publishing to this topic moves the robot. The `Twist` message contains linear and angular velocity components.
-   **Usage Example (CLI):**
    ```bash
    # Move the robot forward at 0.5 m/s
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```

## Request/Response Endpoints (ROS 2 Services)

Services are endpoints for synchronous, request/response interactions, similar to a typical `POST` request in a REST API.

### `POST /spawn_entity`

Spawns a new model into the Gazebo simulation.

-   **Service Name:** `/spawn_entity`
-   **Service Type:** `gazebo_msgs/srv/SpawnEntity`
-   **Description:** Takes a name and URDF/SDF model description and spawns it in the simulation.
-   **Usage Example (CLI):**
    ```bash
    ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'my_box', xml: '<?xml version=...?>'}"
    ```

### `POST /move_to_pose`

Commands the robot's arm to move to a specific Cartesian pose.

-   **Service Name:** `/move_to_pose`
-   **Service Type:** `my_interfaces/srv/SetPose` (hypothetical)
-   **Description:** A blocking call that moves the arm to the desired `(x, y, z)` position and `(w, x, y, z)` orientation. Returns a success boolean.
-   **Usage Example (CLI):**
    ```bash
    ros2 service call /move_to_pose my_interfaces/srv/SetPose "{pose: {position: {x: 0.5, ...}, orientation: {...}}}"
    ```

## Long-Running Task Endpoints (ROS 2 Actions)

Actions are for long-running, asynchronous tasks that provide continuous feedback and can be preempted. This is similar to initiating a job that returns a job ID for status checks.

### `POST /navigate_to_pose`

Commands the robot to navigate to a goal pose in the map.

-   **Action Name:** `/navigate_to_pose`
-   **Action Type:** `nav2_msgs/action/NavigateToPose`
-   **Description:** This action initiates the Nav2 stack to plan and execute a path.
-   **Feedback:** Provides feedback on the robot's current position as it travels.
-   **Result:** Returns the final result (e.g., success, failure).
-   **Usage Example (CLI):**
    ```bash
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
    ```
