# Setup Guide for Robot Arm Project

This guide will walk you through the process of setting up and running the 5-DOF robotic arm simulation from scratch.

> [!NOTE]
> This guide assumes you are using Ubuntu 20.04 with ROS Noetic. Adjustments may be needed for other environments.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Building the Project](#building-the-project)
4. [Running the Simulation](#running-the-simulation)
5. [Exploring the Robot](#exploring-the-robot)
6. [Customizing the Robot](#customizing-the-robot)

## Prerequisites

Ensure your system has the following installed:

- Ubuntu 20.04 (Focal Fossa)
- ROS Noetic
- Gazebo 11
- Git

If you need to install ROS Noetic, follow these steps:

```bash
# Setup ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package lists
sudo apt update

# Install ROS Noetic
sudo apt install ros-noetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

Install additional dependencies:

```bash
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

## Installation

1. Create a workspace for the robot arm project:

   ```bash
   mkdir -p ~/projects/ROS_project/src
   cd ~/projects/ROS_project/src
   ```

2. Clone the repository:

   ```bash
   # Replace with your actual repository URL
   git clone https://github.com/yourusername/robot_arm_project.git .
   ```

3. Install project-specific dependencies:

   ```bash
   cd ~/projects/ROS_project
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Building the Project

Compile the project using catkin build:

```bash
cd ~/projects/ROS_project
catkin build
```

> [!TIP]
> If you encounter build errors, try cleaning the build first:
>
> ```bash
> catkin clean
> catkin build
> ```

Source the workspace:

```bash
source ~/projects/ROS_project/devel/setup.zsh
```

## Running the Simulation

1. Launch the full robot arm simulation:

    ```bash
    roslaunch move full_robot_arm_sim.launch
    ```

    This will start:

    - Gazebo with the robot model
    - MoveIt move_group
    - RViz for visualization

2. In a new terminal window, run the pick and place demo:

```bash
source ~/projects/ROS_project/devel/setup.zsh
rosrun move code.py
```

> [!IMPORTANT]
> Make sure the simulation is fully loaded before starting the demo script.

## Exploring the Robot

### Examining the Robot Model

View the URDF model of the robot:

```bash
rosrun urdf_to_graphiz urdf_to_graphiz arm_assem_urdf_v3
```

### Testing Joint Movement

Move individual joints using RViz:

1. In RViz, go to the "Motion Planning" panel
2. Select "Planning" tab
3. Use the interactive markers to move the robot
4. Click "Plan and Execute" to move the robot

### Understanding the Controllers

List available controllers:

```bash
rosservice call /controller_manager/list_controllers
```

Check controller status:

```bash
rostopic echo /arm_controller/state
```

## Customizing the Robot

### Modifying Controller Parameters

To adjust the controller parameters:

1. Edit the controller configuration file:

    ```bash
    code ~/projects/ROS_project/move/config/ros_controllers.yaml
    ```

2. Modify the PID gains to adjust control performance:

```yaml
gains:
  joint1:
    p: 100 # Increase for more responsive movement
    d: 1 # Increase to reduce oscillations
    i: 1 # Increase for steady-state error reduction
    i_clamp: 1
```

### Adjusting MoveIt Parameters

To change motion planning parameters:

1. Edit the MoveIt configuration:

    ```bash
    code ~/projects/ROS_project/move/config/ompl_planning.yaml
    ```

2. Modify planning parameters:

```yaml
planner_configs:
  RRTConnectkConfigDefault:
    range: 0.0 # Connection distance
```

### Adding New Predefined Poses

To add new robot poses:

1. Use the MoveIt Setup Assistant:

    ```bash
    roslaunch moveit_setup_assistant setup_assistant.launch
    ```

2. Select "Load Files" and browse to your URDF file
3. Go to "Robot Poses" tab and add new poses
4. Save the configuration

### Customizing the Demo Script

Modify the pick and place routine by editing the `code.py` file:

```bash
code ~/projects/ROS_project/move/scripts/code.py
```

Example modification to add a new pose:

```python
# Add a new position in the sequence
arm.set_pose("zero")
rospy.sleep(1)
arm.set_pose("pick")
rospy.sleep(1)
arm.set_pose("my_new_pose")  # Add your new pose here
rospy.sleep(1)
```

## Troubleshooting

If you encounter issues, refer to the TROUBLESHOOTING_GUIDE.md file for detailed solutions to common problems.

For more advanced documentation, see:

- [DOCUMENTATION.md](./DOCUMENTATION.md): General project documentation
- [TECHNICAL_DOCUMENTATION.md](./TECHNICAL_DOCUMENTATION.md): Technical details about the implementation
