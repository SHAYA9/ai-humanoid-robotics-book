# SpecifyPlus Commands Reference

This document provides a comprehensive list of SpecifyPlus commands for the AI Humanoid Robotics Book project. These commands follow a structured, specification-driven approach to development.

## Command Structure

All SpecifyPlus commands follow this format:

```
/spec <module> <action> <component> [options]
```

- **module**: The course module (ros2, simulation, isaac, vla)
- **action**: The operation (create, update, test, deploy, document)
- **component**: The specific component to work with
- **options**: Additional parameters

---

## Module 1: ROS 2 - The Robotic Nervous System

### Create ROS 2 Components

#### Create a new ROS 2 package
```
/spec ros2 create package <package_name> --type <python|cpp> --dependencies <dep1,dep2>
```

**Example:**
```
/spec ros2 create package humanoid_controller --type python --dependencies rclpy,std_msgs,sensor_msgs
```

**Specification:**
- Creates a new ROS 2 package with proper structure
- Includes setup.py/CMakeLists.txt configuration
- Adds specified dependencies
- Creates basic package.xml with metadata

#### Create a ROS 2 Node
```
/spec ros2 create node <node_name> --package <package_name> --type <publisher|subscriber|service|action>
```

**Example:**
```
/spec ros2 create node joint_state_publisher --package humanoid_controller --type publisher --topic /joint_states --msg-type sensor_msgs/JointState
```

**Specification:**
- Creates a Python/C++ node file
- Implements specified communication pattern
- Adds proper logging and error handling
- Includes entry point in setup.py

#### Create URDF Model
```
/spec ros2 create urdf <robot_name> --base-type <humanoid|quadruped|mobile> --dof <number>
```

**Example:**
```
/spec ros2 create urdf atlas_humanoid --base-type humanoid --dof 28 --height 1.8m --sensors camera,imu,lidar
```

**Specification:**
- Generates URDF file with proper link/joint structure
- Includes collision and visual meshes
- Adds sensor definitions (camera, IMU, LiDAR)
- Creates launch file for visualization in RViz

### Update ROS 2 Components

#### Update Node Implementation
```
/spec ros2 update node <node_name> --add-subscriber <topic> --add-publisher <topic> --add-service <service>
```

**Example:**
```
/spec ros2 update node joint_state_publisher --add-subscriber /cmd_vel --add-timer 0.1
```

#### Update URDF
```
/spec ros2 update urdf <robot_name> --add-link <link_name> --add-joint <joint_name> --add-sensor <sensor_type>
```

### Test ROS 2 Components

#### Test Node
```
/spec ros2 test node <node_name> --unit --integration --coverage
```

**Example:**
```
/spec ros2 test node joint_state_publisher --unit --integration --mock-topics /cmd_vel
```

**Specification:**
- Creates unit tests using pytest
- Creates integration tests with mock topics
- Generates coverage report
- Validates message types and frequencies

#### Test URDF
```
/spec ros2 test urdf <robot_name> --check-validity --visualize --collision-check
```

---

## Module 2: Simulation - The Digital Twin

### Create Simulation Components

#### Create Gazebo World
```
/spec simulation create world <world_name> --environment <indoor|outdoor|warehouse> --size <dimensions>
```

**Example:**
```
/spec simulation create world robotics_lab --environment indoor --size 10x10x3 --objects table,chair,obstacles --lighting natural
```

**Specification:**
- Creates .world file with specified environment
- Adds physics parameters (gravity, friction)
- Includes lighting configuration
- Adds static objects and obstacles

#### Create Unity Scene
```
/spec simulation create unity-scene <scene_name> --quality <low|medium|high|ultra> --sensors <sensor_list>
```

**Example:**
```
/spec simulation create unity-scene photorealistic_lab --quality ultra --sensors rgb_camera,depth_camera --physics-engine nvidia-physx
```

#### Create Sensor Simulation
```
/spec simulation create sensor <sensor_type> --parent-link <link_name> --frequency <hz> --resolution <spec>
```

**Example:**
```
/spec simulation create sensor camera --parent-link head_link --frequency 30 --resolution 1920x1080 --fov 90
/spec simulation create sensor lidar --parent-link base_link --frequency 10 --range 30m --rays 360
```

### Launch Simulation

#### Launch Gazebo Simulation
```
/spec simulation launch gazebo --world <world_name> --robot <urdf_file> --gui --rviz
```

**Example:**
```
/spec simulation launch gazebo --world robotics_lab --robot atlas_humanoid.urdf --gui --rviz --record-bag
```

#### Launch Unity Simulation
```
/spec simulation launch unity --scene <scene_name> --robot <urdf_file> --ros-bridge
```

### Test Simulation

#### Test Physics
```
/spec simulation test physics --world <world_name> --scenarios <scenario_list>
```

**Example:**
```
/spec simulation test physics --world robotics_lab --scenarios walking,falling,collision --validate-contacts
```

---

## Module 3: NVIDIA Isaac - The AI-Robot Brain

### Create Isaac Components

#### Create Isaac ROS Node
```
/spec isaac create ros-node <node_name> --type <vslam|depth|apriltag|object-detection>
```

**Example:**
```
/spec isaac create ros-node visual_slam --type vslam --input-topic /camera/image_raw --output-topic /slam/pose
```

**Specification:**
- Creates Isaac ROS node configuration
- Sets up GPU acceleration
- Configures input/output topics
- Adds performance monitoring

#### Create Isaac Sim Environment
```
/spec isaac create sim-environment <env_name> --type <warehouse|office|outdoor> --assets <asset_list>
```

**Example:**
```
/spec isaac create sim-environment training_warehouse --type warehouse --assets shelves,boxes,pallets --randomize-layout
```

#### Create Navigation Stack
```
/spec isaac create navigation --planner <nav2|isaac-nav> --costmap-config <config_file>
```

**Example:**
```
/spec isaac create navigation --planner nav2 --costmap-config bipedal_costmap.yaml --local-planner dwa --global-planner dijkstra
```

**Specification:**
- Creates Nav2 configuration for bipedal locomotion
- Sets up costmap layers (static, inflation, voxel)
- Configures planners for humanoid constraints
- Adds recovery behaviors

### Train Isaac Models

#### Train Perception Model
```
/spec isaac train perception --model <model_type> --dataset <dataset_path> --epochs <num> --gpu <id>
```

**Example:**
```
/spec isaac train perception --model object-detection --dataset /data/warehouse_objects --epochs 100 --gpu 0 --batch-size 32
```

#### Generate Synthetic Data
```
/spec isaac generate data --scene <scene_name> --num-samples <num> --randomize <params>
```

**Example:**
```
/spec isaac generate data --scene training_warehouse --num-samples 10000 --randomize lighting,pose,occlusion --output /data/synthetic
```

---

## Module 4: VLA - Vision-Language-Action

### Create VLA Components

#### Create Voice Interface
```
/spec vla create voice-interface --model <whisper-tiny|whisper-base|whisper-large> --language <lang>
```

**Example:**
```
/spec vla create voice-interface --model whisper-base --language en --output-topic /voice/transcription --continuous
```

**Specification:**
- Integrates OpenAI Whisper for speech-to-text
- Creates ROS 2 node publishing transcriptions
- Adds voice activity detection
- Implements continuous listening mode

#### Create LLM Planner
```
/spec vla create llm-planner --model <gpt-4|claude|gemini> --system-prompt <prompt_file> --tools <tool_list>
```

**Example:**
```
/spec vla create llm-planner --model gemini-2.0-flash --system-prompt robotics_assistant.txt --tools navigation,manipulation,perception
```

**Specification:**
- Creates LLM integration node
- Defines system prompt for robotics tasks
- Implements tool calling for robot actions
- Adds conversation history management

#### Create Multimodal Integration
```
/spec vla create multimodal --vision-model <model> --language-model <model> --action-space <config>
```

**Example:**
```
/spec vla create multimodal --vision-model clip --language-model gemini-2.0-flash --action-space humanoid_actions.yaml --fusion-strategy late
```

**Specification:**
- Integrates vision and language models
- Creates unified action space
- Implements multimodal fusion
- Adds grounding mechanisms

### Create Autonomous Behaviors

#### Create Task Executor
```
/spec vla create task-executor --tasks <task_list> --fallback-strategy <strategy>
```

**Example:**
```
/spec vla create task-executor --tasks pick,place,navigate,search --fallback-strategy retry --max-retries 3
```

---

## Cross-Module Commands

### Documentation

#### Generate Documentation
```
/spec document generate --module <module_name> --format <markdown|html|pdf> --include <sections>
```

**Example:**
```
/spec document generate --module all --format markdown --include api,architecture,tutorials --output docs/
```

#### Create Tutorial
```
/spec document create tutorial --title <title> --module <module> --difficulty <level>
```

**Example:**
```
/spec document create tutorial --title "Building Your First Humanoid Controller" --module ros2 --difficulty beginner --steps 10
```

### Deployment

#### Deploy to Cloud
```
/spec deploy cloud --platform <railway|aws|gcp> --services <service_list> --env <env_file>
```

**Example:**
```
/spec deploy cloud --platform railway --services backend,chatbot --env .env.production --auto-scale
```

#### Deploy to Robot
```
/spec deploy robot --target <robot_ip> --packages <package_list> --auto-start
```

**Example:**
```
/spec deploy robot --target 192.168.1.100 --packages humanoid_controller,navigation_stack --auto-start --monitor
```

### Testing

#### Run Integration Tests
```
/spec test integration --modules <module_list> --scenarios <scenario_file>
```

**Example:**
```
/spec test integration --modules ros2,simulation,isaac --scenarios warehouse_navigation.yaml --record-results
```

#### Run System Tests
```
/spec test system --full-stack --hardware <real|simulated> --duration <time>
```

**Example:**
```
/spec test system --full-stack --hardware simulated --duration 1h --scenarios pick_and_place,navigation,voice_control
```

---

## Project Management Commands

### Initialize Project
```
/spec project init --name <project_name> --modules <module_list> --template <template>
```

**Example:**
```
/spec project init --name my_humanoid_robot --modules ros2,simulation,isaac,vla --template full-stack
```

### Create Feature
```
/spec project create feature --name <feature_name> --module <module> --spec <spec_file>
```

**Example:**
```
/spec project create feature --name autonomous_grasping --module vla --spec grasping_spec.yaml --tests --docs
```

### Update Dependencies
```
/spec project update deps --check-compatibility --auto-upgrade --test
```

### Generate Report
```
/spec project report --type <progress|performance|coverage> --format <format> --period <period>
```

**Example:**
```
/spec project report --type progress --format pdf --period weekly --include metrics,tests,commits
```

---

## Best Practices

1. **Always specify dependencies** when creating packages
2. **Include tests** with every component creation
3. **Document as you build** using the document commands
4. **Use integration tests** to validate cross-module functionality
5. **Version control** all generated specifications
6. **Review generated code** before committing
7. **Use templates** for consistent structure
8. **Monitor performance** during development

---

## Configuration Files

SpecifyPlus uses YAML configuration files for complex specifications:

### Example: Robot Specification
```yaml
# robot_spec.yaml
robot:
  name: atlas_humanoid
  type: humanoid
  dof: 28
  height: 1.8m
  weight: 80kg
  
  links:
    - name: base_link
      type: box
      dimensions: [0.3, 0.3, 0.5]
    - name: head_link
      type: sphere
      radius: 0.15
      
  joints:
    - name: waist_joint
      type: revolute
      parent: base_link
      child: torso_link
      axis: [0, 0, 1]
      limits: [-1.57, 1.57]
      
  sensors:
    - name: head_camera
      type: camera
      parent: head_link
      resolution: [1920, 1080]
      fov: 90
      frequency: 30
```

### Example: Task Specification
```yaml
# task_spec.yaml
task:
  name: pick_and_place
  description: "Pick up object and place it on table"
  
  preconditions:
    - robot_initialized
    - object_detected
    - table_detected
    
  steps:
    - action: navigate
      target: object_location
      tolerance: 0.1m
      
    - action: grasp
      object: target_object
      approach: top_down
      force: 10N
      
    - action: navigate
      target: table_location
      tolerance: 0.1m
      
    - action: place
      location: table_surface
      release_height: 0.05m
      
  postconditions:
    - object_on_table
    - gripper_empty
```

---

## Command Aliases

For frequently used commands, SpecifyPlus supports aliases:

```bash
# Create alias
/spec alias create build-all "spec project build --all --tests --docs"

# Use alias
/spec build-all
```

---

## Help and Documentation

```
/spec help <command>          # Get help for specific command
/spec list commands           # List all available commands
/spec list modules            # List all modules
/spec examples <module>       # Show examples for module
```