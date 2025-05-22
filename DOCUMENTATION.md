# 5-DOF Robot Arm with Gripper - Documentation

![Robot Arm](media/images/robot_arm.png)

## Table of Contents

1. [Project Overview](#project-overview)
2. [Hardware Components](#hardware-components)
3. [Software Architecture](#software-architecture)
4. [Installation and Setup](#installation-and-setup)
5. [Running the Robot](#running-the-robot)
6. [Robot Functionality](#robot-functionality)
7. [Common Issues and Solutions](#common-issues-and-solutions)
8. [Future Improvements](#future-improvements)
9. [Video Demos](#video-demos)

## Project Overview

This project involves the development and implementation of a 5-DOF robotic arm with a gripper capable of performing pick and place operations in a Gazebo simulation environment. The system leverages ROS (Robot Operating System) and MoveIt for motion planning and control.

> [!NOTE]
> This robot arm is designed for educational purposes and demonstrates fundamental principles of robotic manipulation, motion planning, and control.

## Hardware Components

### Robot Structure

The robot consists of the following components:

1. **Base**: Fixed base serving as the foundation for the robot
2. **Links**:

   - Link1: First joint rotation (connects to base)
   - Link2: Second joint for arm extension
   - Link3: Third joint for arm articulation
   - Link4: Fourth joint for wrist positioning
   - Link5: Fifth joint for gripper base orientation

3. **Gripper System**:
   - rightGripper & leftGripper: Gripper mechanism
   - r_finger & l_fing: Finger components for object manipulation

### Coordinate System

The robot uses a standard right-handed coordinate system:

- X-axis: Forward direction
- Y-axis: Left direction
- Z-axis: Up direction

## Software Architecture

### System Components

1. **ROS Framework**: Provides communication infrastructure, message passing, and service calls
2. **MoveIt**: Handles motion planning, kinematics, and collision checking
3. **Gazebo**: Physics simulation environment

### Key Packages

1. **arm_assem_urdf_v3**: Contains URDF model, meshes, and launch files for the robot
2. **move**: Contains MoveIt configurations, scripts, and launch files for controlling the robot

### Control Architecture

The robot uses a position-based control system with:

- Position controllers for joint trajectory execution
- Action-based interfaces for high-level control
- Predefined poses for task execution

## Installation and Setup

### Prerequisites

- Ubuntu 20.04 (Recommended)
- ROS Noetic
- Gazebo 11
- MoveIt

### Installation Steps

1. Install ROS and dependencies:

    ```bash
    sudo apt-get update
    sudo apt-get install ros-noetic-desktop-full
    sudo apt-get install ros-noetic-moveit
    sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
    ```

2. Create a workspace and clone the project:

    ```bash
    mkdir -p ~/projects/ROS_project/src
    cd ~/projects/ROS_project/src
    git clone https://github.com/AhmedKhaledp-0/ROS_project.git
    cd ..
    ```

3. Build the project:

```bash
catkin build
source devel/setup.bash
```

## Running the Robot

### Starting the Simulation

To launch the full robot arm simulation with Gazebo, MoveIt, and RViz:

```bash
source devel/setup.bash
roslaunch move full_robot_arm_sim.launch
```

> [!TIP]
> Ensure that your environment variables are properly set by checking the output of `echo $ROS_PACKAGE_PATH` to confirm it includes your workspace.

### Running the Pick and Place Demo

To execute the pick and place demo:

```bash
source devel/setup.bash
rosrun move code.py
```

## Robot Functionality

### Motion Sequence

The robot performs a pick and place cycle with the following steps:

1. **Initialize Position**:

   - Arm moves to the "zero" position
   - Gripper opens ("zero" position)

2. **Pick Operation**:

   - Arm moves to "pick" position over the object
   - Gripper closes to grab the object

3. **Transport Operation**:

   - Arm lifts object ("lift" position)
   - Arm rotates to target location ("turn" position)

4. **Place Operation**:

   - Arm moves to "drop" position
   - Gripper opens to release the object

5. **Repeat**: The cycle continues until interrupted

### Predefined Poses

The robot has several predefined poses configured in the SRDF file:

1. **Arm Group**:

   - **zero**: Home position (joint1: -1.57, joint2: -0.3268, joint3: 0.0159, joint4: 0.263, joint5: -0.0797)
   - **pick**: Position for picking objects (joint1: -1.57, joint2: 1.0599, joint3: -0.4702, joint4: -0.6296, joint5: -0.0797)
   - **lift**: Position for lifting objects (same as zero position)
   - **turn**: Position for turning toward drop location (joint1: 1.57, other joints same as zero)
   - **drop**: Position for dropping objects (joint1: 1.57, other joints same as pick position)

2. **Hand Group**:
   - **zero**: Initial gripper position
   - **open**: Open gripper (Lgripperjoint: 0.6, rgripper_joint: 0.6)
   - **close**: Closed gripper (Lgripperjoint: -0.3, rgripper_joint: -0.3)

## Common Issues and Solutions

### Collision Issues

> [!CAUTION]
> Incorrect collision settings can cause simulation crashes or unrealistic behavior.

The collision properties of the gripper components need to be properly configured to avoid self-collision and ensure realistic physics:

```xml
<gazebo reference="rightGripper">
  <selfCollide>false</selfCollide>
  <material>Gazebo/Black</material>
</gazebo>

<gazebo reference="r_finger">
  <selfCollide>false</selfCollide>
  <material>Gazebo/Black</material>
</gazebo>

<gazebo reference="leftGripper">
  <selfCollide>false</selfCollide>
  <material>Gazebo/Black</material>
</gazebo>

<gazebo reference="l_fing">
  <selfCollide>false</selfCollide>
  <material>Gazebo/Black</material>
</gazebo>
```

Additionally, disable collision between gripper parts that should not interact:

```xml
<disable_collision>rightGripper leftGripper</disable_collision>
<disable_collision>rightGripper l_fing</disable_collision>
<disable_collision>leftGripper r_finger</disable_collision>
<disable_collision>r_finger l_fing</disable_collision>
```

### MoveIt Velocity Tolerance Issues

> [!IMPORTANT]
> Proper velocity tolerance settings are crucial for smooth movement transitions.

When configuring the MoveIt controllers, ensure appropriate tolerance values:

```yaml
controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
    constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.7
      joint1: { trajectory: 0.1, goal: 1 }
      joint2: { trajectory: 0.1, goal: 1 }
      joint3: { trajectory: 0.1, goal: 1 }
      joint4: { trajectory: 0.1, goal: 1 }
      joint5: { trajectory: 0.1, goal: 1 }
```

> [!WARNING]
> If the `stopped_velocity_tolerance` is too low, the robot may never reach its goal position due to small oscillations. If it's too high, the robot might not fully stop before attempting the next motion.

### Solutions to Common Problems

1. **Gazebo Physics Issues**:

   - Increase physics update rate in Gazebo
   - Adjust mass and inertia properties in the URDF

2. **Planning Issues**:

   - Increase planning time allowed
   - Try different planners (OMPL, CHOMP, STOMP)
   - Adjust goal tolerance parameters

3. **Controller Issues**:
   - Tune PID parameters in controller configuration
   - Adjust joint limits and velocity constraints

## Future Improvements

1. **Advanced Perception**:

   - Add camera sensors for object recognition
   - Implement computer vision algorithms

2. **Dynamic Motion Planning**:

   - Add obstacle avoidance capabilities
   - Implement online trajectory planning

3. **Enhanced Gripper**:
   - Implement force feedback
   - Add tactile sensors

## Video Demos

Watch the robot arm demo video: [Robot Arm Demo](media/videos/robot.mp4)

This video demonstrates the complete pick and place operation of the robot arm in the Gazebo simulation environment.
