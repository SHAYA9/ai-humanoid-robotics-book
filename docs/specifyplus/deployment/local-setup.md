# Deployment: Local Setup

This guide details the steps required to set up the complete SpecifyPlus development environment on a local machine. The primary target is a Linux system (Ubuntu 22.04) with a compatible NVIDIA GPU.

## Step 1: Install ROS 2

ROS 2 Humble Hawksbill is the foundation of the system. Follow the official ROS 2 documentation to install it.

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-gz

# Source the setup script
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 2: Install Gazebo Simulator

Gazebo (or Ignition Gazebo) is the recommended physics simulator. `ros-humble-ros-gz` will install the necessary packages.

To test the installation, run:

```bash
gz sim -v4 shapes.sdf
```

This should open a Gazebo window with several simple shapes.

## Step 3: Install NVIDIA GPU Drivers and CUDA

An NVIDIA GPU is required for the Isaac Sim and Isaac ROS components.

1.  **Install Drivers:** Install the latest proprietary NVIDIA drivers for your GPU. This can usually be done through Ubuntu's "Additional Drivers" tool.
2.  **Install CUDA Toolkit:** Download and install the CUDA Toolkit from the NVIDIA developer website. Ensure you select the version compatible with your driver and the Isaac SDK.

Verify the installation:

```bash
nvidia-smi
nvcc --version
```

## Step 4: Install NVIDIA Isaac Sim

Isaac Sim is a powerful, GPU-accelerated simulator.

1.  **Install Omniverse Launcher:** Download and install the NVIDIA Omniverse Launcher.
2.  **Install Isaac Sim:** From the Omniverse Launcher's "Exchange" tab, search for "Isaac Sim" and install it. Make sure to install a version that is compatible with your ROS 2 distribution (Humble).
3.  **Enable ROS 2 Bridge:** Once installed, go to `Isaac Sim > Settings > Extensions` and ensure the `omni.isaac.ros2_bridge` extension is enabled.

## Step 5: Set up a SpecifyPlus Workspace

With all the prerequisites installed, you can now set up a ROS 2 workspace to contain the SpecifyPlus packages.

1.  **Create Workspace:**
    ```bash
    mkdir -p ~/specifyplus_ws/src
    cd ~/specifyplus_ws
    ```

2.  **Clone Packages:**
    ```bash
    # (Hypothetical) Clone the necessary repositories for your robot
    git clone https://github.com/my-robot-co/robot_description.git src/robot_description
    git clone https://github.com/my-robot-co/robot_control.git src/robot_control
    ```

3.  **Install Dependencies and Build:**
    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build --symlink-install
    ```

4.  **Source Workspace:**
    ```bash
    echo "source ~/specifyplus_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

Your local setup is now complete. You can launch components of the system using `ros2 launch` commands.
