---
sidebar_position: 4
---

# 8.4 Object Detection and Manipulation

For our robot to act on the world, it needs two key capabilities: **perception** to understand its environment and **manipulation** to interact with it. In our VLA system, these are handled by dedicated ROS 2 nodes that provide their functionality as services to the main planner.

### Vision System: Object Detection

The "find_object" skill in our planner needs a robust backend. This will be a `VisionNode` responsible for identifying and locating objects in the robot's workspace.

-   **Input:** A continuous stream of images from a camera (simulated in our case).
-   **Processing:** The node uses a pre-trained **object detection model**. A popular and effective choice is **YOLO (You Only Look Once)**, which is fast and accurate. The model will process each image and identify bounding boxes for all known objects.
-   **3D Position:** To go from a 2D bounding box in an image to a 3D coordinate in the robot's world, we will use the camera's **depth sensor**. By finding the center of the bounding box and looking up the corresponding distance from the depth map, we can calculate the object's `[x, y, z]` position relative to the camera.
-   **Service:** The `VisionNode` exposes its functionality through a ROS 2 service, for example, `/vision/find_object`.
    -   **Request:** `string object_name` (e.g., "red cube")
    -   **Response:** `geometry_msgs/Point position` (the 3D coordinates) and `bool success`.

### Manipulation System: Motion Planning

Once we have the coordinates of an object, we need to move the robot arm to it safely and efficiently. This is the job of the **manipulation** system, which will be powered by the **MoveIt2** framework.

-   **MoveIt2:** MoveIt2 is the standard motion planning library in ROS 2. It takes a start state, a goal state (e.g., the 6D pose of the end-effector), and a description of the robot and its environment, and it computes a collision-free trajectory for the arm's joints.
-   **Node:** A `MotionNode` will wrap the MoveIt2 functionality.
-   **Service:** It will expose a service like `/motion/move_to_pose`.
    -   **Request:** `geometry_msgs/PoseStamped target_pose` (the desired position and orientation of the gripper).
    -   **Response:** `bool success`.

### Gripper Control

The final piece is the ability to grasp and release objects.

-   **Node:** A simple `GripperNode` will control the robot's end-effector.
-   **Service:** It will provide a service like `/gripper/set_state`.
    -   **Request:** `string state` ("open" or "close").
    -   **Response:** `bool success`.

By encapsulating these complex tasks into discrete services, we create a clean and modular architecture. The main `CognitivePlannerNode` doesn't need to know *how* object detection or motion planning work; it just needs to know how to call the services. This separation of concerns is a hallmark of good robotics software design.
