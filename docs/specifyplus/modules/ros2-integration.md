# Module 1: ROS 2 Integration

This module documentation covers the integration of the Robotic Nervous System (ROS 2) within the SpecifyPlus framework. ROS 2 forms the communication backbone of the entire system.

## ROS 2 Architecture

The SpecifyPlus system leverages the core concepts of ROS 2 to build a distributed and modular robotics application.

-   **Graph:** The system is a network of ROS 2 nodes, which represents the entire software stack. Tools like `rqt_graph` can be used to visualize the connections between different components.
-   **DDS (Data Distribution Service):** Under the hood, ROS 2 uses DDS for its publish-subscribe messaging. This provides a robust, real-time communication layer that is ideal for robotics applications.

## Nodes, Topics, and Services

These are the fundamental building blocks of a SpecifyPlus application.

### Nodes

A node is the smallest unit of computation. Each node in SpecifyPlus is responsible for a single, specific purpose, such as controlling a motor, reading a sensor, or planning a path.

### Topics

Topics are named buses over which nodes exchange messages. SpecifyPlus uses a standardized set of topics for common robotics data.

-   `/joint_states`: Publishes the current angle and velocity of all robot joints.
-   `/scan`: For 2D LiDAR data.
-   `/camera/image_raw`: Raw image data from a camera.
-   `/cmd_vel`: Used to send velocity commands to the robot's base.

**Example: Listening to a Topic**

```bash
ros2 topic echo /joint_states
```

### Services

Services are used for request/response communication. Unlike topics, services are synchronous and are used for tasks that have a clear start and end.

**Example: Calling a Service**

```bash
# Hypothetical service to set the robot's arm to a specific pose
ros2 service call /move_arm_to_pose my_interfaces/srv/SetPose "{position: {x: 0.5, y: 0.2, z: 1.0}}"
```

## rclpy Python Agents

The logic for most nodes in SpecifyPlus is written in Python using the `rclpy` client library. These "agents" are Python scripts that can create nodes, publishers, subscribers, and services.

**Example: A Simple Subscriber Agent**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states for: {msg.name}')

def main(args=None):
    rclpy.init(args=args)
    monitor = JointStateMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## URDF for Humanoids

The physical model of the humanoid robot is defined using the Unified Robot Description Format (URDF). The URDF is an XML file that describes:

-   **Links:** The rigid parts of the robot (e.g., torso, upper arm, gripper).
-   **Joints:** The connections between links, defining how they can move relative to each other (e.g., revolute, prismatic).
-   **Visuals:** The 3D mesh and appearance of each link.
-   **Collisions:** The collision geometry used by the physics engine.

This URDF file is loaded by the `robot_state_publisher` node, which broadcasts the transforms of all links, and is also used by the simulator (Gazebo) to create the robot's digital twin.

**Example URDF Snippet:**

```xml
<robot name="my_humanoid">
  <link name="torso">
    <visual>
      <geometry>
        <mesh filename="package://my_robot_description/meshes/torso.dae"/>
      </geometry>
    </visual>
  </link>

  <link name="upper_arm_l"/>

  <joint name="shoulder_l_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm_l"/>
    <origin xyz="0.0 0.2 0.5"/>
    <axis xyz="0 1 0"/>
    <limit effort="100.0" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
</robot>
```
