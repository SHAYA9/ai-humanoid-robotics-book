---
id: tutorials
title: "Step-by-Step Tutorials"
sidebar_label: "Tutorials"
description: "Hands-on tutorials for building real robotics projects"
tags: [tutorials, hands-on, projects, examples]
---

# Step-by-Step Tutorials

Learn by doing with these comprehensive, hands-on tutorials. Each tutorial builds a complete project from scratch.

## 🎯 Tutorial Categories

### 🟢 Beginner Tutorials
- [Tutorial 1: Your First ROS 2 Robot](#tutorial-1-your-first-ros-2-robot)
- [Tutorial 2: Build a Simulated Mobile Robot](#tutorial-2-build-a-simulated-mobile-robot)
- [Tutorial 3: Create a URDF Humanoid Model](#tutorial-3-create-a-urdf-humanoid-model)

### 🟡 Intermediate Tutorials
- [Tutorial 4: Autonomous Navigation with Nav2](#tutorial-4-autonomous-navigation-with-nav2)
- [Tutorial 5: Object Detection with Isaac ROS](#tutorial-5-object-detection-with-isaac-ros)
- [Tutorial 6: Voice-Controlled Robot](#tutorial-6-voice-controlled-robot)

### 🔴 Advanced Tutorials
- [Tutorial 7: VLA-Powered Autonomous Humanoid](#tutorial-7-vla-powered-autonomous-humanoid)
- [Tutorial 8: Sim-to-Real Transfer](#tutorial-8-sim-to-real-transfer)
- [Tutorial 9: Multi-Robot Coordination](#tutorial-9-multi-robot-coordination)

---

## Tutorial 1: Your First ROS 2 Robot

**Duration:** 2 hours  
**Difficulty:** 🟢 Beginner  
**What you'll build:** A simple robot that moves in a square pattern

### Prerequisites
- Ubuntu 22.04 installed
- ROS 2 Humble installed
- Basic Python knowledge

### Step 1: Create Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python my_first_robot \
  --dependencies rclpy geometry_msgs
```

### Step 2: Write the Robot Controller

Create `my_first_robot/my_first_robot/square_mover.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Square Mover Node Started!')
        
    def move_forward(self, distance, speed=0.2):
        """Move forward for a given distance"""
        msg = Twist()
        msg.linear.x = speed
        
        # Calculate time needed
        duration = distance / speed
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
            
        # Stop
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        
    def turn(self, angle, angular_speed=0.5):
        """Turn by a given angle (in degrees)"""
        msg = Twist()
        msg.angular.z = angular_speed
        
        # Convert angle to radians and calculate time
        import math
        angle_rad = math.radians(angle)
        duration = abs(angle_rad / angular_speed)
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
            
        # Stop
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        
    def move_square(self, side_length=1.0):
        """Move in a square pattern"""
        self.get_logger().info('Starting square movement...')
        
        for i in range(4):
            self.get_logger().info(f'Side {i+1}/4')
            self.move_forward(side_length)
            time.sleep(0.5)
            self.turn(90)
            time.sleep(0.5)
            
        self.get_logger().info('Square completed!')

def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    
    try:
        node.move_square(side_length=2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Update setup.py

Edit `setup.py` to add the entry point:

```python
entry_points={
    'console_scripts': [
        'square_mover = my_first_robot.square_mover:main',
    ],
},
```

### Step 4: Build and Run

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select my_first_robot

# Source
source install/setup.bash

# Run
ros2 run my_first_robot square_mover
```

### Step 5: Visualize in RViz

In another terminal:

```bash
# Launch RViz
rviz2
```

### 🎉 Success Criteria
- ✅ Node starts without errors
- ✅ Robot moves in square pattern
- ✅ Logs show progress
- ✅ Can visualize in RViz

### 🚀 Next Steps
- Modify to move in different patterns (circle, figure-8)
- Add obstacle avoidance
- Create a launch file
- Add parameters for customization

---

## Tutorial 2: Build a Simulated Mobile Robot

**Duration:** 3 hours  
**Difficulty:** 🟢 Beginner  
**What you'll build:** A differential drive robot in Gazebo

### What You'll Learn
- URDF robot modeling
- Gazebo integration
- Sensor simulation
- Teleoperation

### Project Structure
```
mobile_robot/
├── urdf/
│   ├── mobile_robot.urdf
│   └── mobile_robot.gazebo
├── launch/
│   ├── gazebo.launch.py
│   └── rviz.launch.py
├── config/
│   └── rviz_config.rviz
└── worlds/
    └── test_world.world
```

### Step 1: Create URDF Model

`urdf/mobile_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="mobile_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" 
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" 
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (similar to left) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" 
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster Wheel -->
  <link name="caster">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" 
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="-0.2 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR Sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" 
               iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
```

### Step 2: Add Gazebo Plugins

`urdf/mobile_robot.gazebo`:

```xml
<?xml version="1.0"?>
<robot>
  <!-- Differential Drive Plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <update_rate>50</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- LiDAR Plugin -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.2</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
        <topic_name>/scan</topic_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Step 3: Create Launch File

`launch/gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 
                           'launch', 'gazebo.launch.py')
            ]),
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'mobile_robot', 
                      '-topic', 'robot_description'],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
    ])
```

### Step 4: Test the Robot

```bash
# Build
colcon build --packages-select mobile_robot

# Source
source install/setup.bash

# Launch Gazebo
ros2 launch mobile_robot gazebo.launch.py

# In another terminal, control the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 🎉 Success Criteria
- ✅ Robot spawns in Gazebo
- ✅ Can control with keyboard
- ✅ LiDAR data visible
- ✅ Odometry published

### 📚 Learning Points
1. **URDF Structure**: Links, joints, and their relationships
2. **Gazebo Plugins**: How to add sensors and actuators
3. **ROS 2 Integration**: Topics, transforms, and state publishing
4. **Simulation Testing**: Before deploying to real hardware

### 🚀 Challenges
Try these extensions:
1. Add a camera sensor
2. Create obstacles in the world
3. Implement autonomous obstacle avoidance
4. Add IMU sensor for better odometry

---

## Tutorial 3: Create a URDF Humanoid Model

**Duration:** 4 hours  
**Difficulty:** 🟢 Beginner  
**What you'll build:** A simplified humanoid robot model

### Humanoid Structure

```
        head
         |
    ----torso----
    |           |
  left_arm   right_arm
    |           |
  left_hand  right_hand
         |
    ----pelvis----
    |            |
  left_leg    right_leg
    |            |
  left_foot  right_foot
```

### Step 1: Define Base Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso (main body) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" 
               iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" 
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Continue with arms, legs, etc. -->
</robot>
```

### Full tutorial continues with:
- Arms and hands
- Legs and feet
- Joint limits and constraints
- Adding cameras and sensors
- Testing in Gazebo
- Creating walking animations

---

## 🎯 More Tutorials Coming Soon

- Tutorial 4: Autonomous Navigation with Nav2
- Tutorial 5: Object Detection with Isaac ROS
- Tutorial 6: Voice-Controlled Robot
- Tutorial 7: VLA-Powered Autonomous Humanoid
- Tutorial 8: Sim-to-Real Transfer
- Tutorial 9: Multi-Robot Coordination

---

## 📺 Video Tutorials

Each tutorial has an accompanying video walkthrough:
- **YouTube Playlist**: [Physical AI Tutorials](https://youtube.com/playlist/...)
- **Duration**: 15-30 minutes per tutorial
- **Format**: Screen recording + explanation

---

## 💬 Get Help

Stuck on a tutorial?
- **Discord**: #tutorials channel
- **Forum**: Post your question with tutorial number
- **Office Hours**: Tuesday & Thursday 3-4 PM EST

---

**Ready to build?** Start with [Tutorial 1](#tutorial-1-your-first-ros-2-robot)!