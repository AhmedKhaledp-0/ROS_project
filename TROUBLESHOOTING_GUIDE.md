# Troubleshooting Guide for Robot Arm Project

> [!IMPORTANT]
> This guide addresses common issues encountered when working with the 4-DOF robot arm simulation. Follow these solutions to resolve typical problems.

## Table of Contents

1. [Collision Issues](#collision-issues)
2. [Gazebo Simulation Problems](#gazebo-simulation-problems)
3. [MoveIt Configuration Problems](#moveit-configuration-problems)
4. [Controller-Related Issues](#controller-related-issues)
5. [Build and Package Problems](#build-and-package-problems)

## Collision Issues

### Symptom: Robot gripper components collide unrealistically or cause simulation instability

> [!CAUTION]
> Incorrect collision settings can cause Gazebo to crash or behave erratically!

### Solution 1: Update Gazebo collision properties

Check and modify the collision properties in the URDF file:

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

### Solution 2: Add collision exclusions in SRDF

Add these lines to your `arm_assem_urdf_v3.srdf` file:

```xml
<disable_collision>rightGripper leftGripper</disable_collision>
<disable_collision>rightGripper l_fing</disable_collision>
<disable_collision>leftGripper r_finger</disable_collision>
<disable_collision>r_finger l_fing</disable_collision>
```

## Gazebo Simulation Problems

### Symptom: Gazebo crashes or freezes when running simulation

> [!WARNING]
> Gazebo can be resource-intensive and may encounter physics engine issues with complex models.

### Solution 1: Reduce physics update rate

Add the following to your Gazebo launch file:

```xml
<param name="/gazebo/physics/max_update_rate" value="100.0"/>
<param name="/gazebo/physics/time_step" value="0.001"/>
```

### Solution 2: Simplify collision meshes

Use simplified collision meshes (primitives like boxes, cylinders) instead of detailed meshes for collision checking:

```xml
<collision>
  <geometry>
    <cylinder length="0.1" radius="0.02"/>
  </geometry>
</collision>
```

## MoveIt Configuration Problems

### Symptom: Robot cannot reach target positions or moves erratically

> [!NOTE]
> MoveIt requires precise configuration of tolerances and planning parameters.

### Solution 1: Adjust trajectory tolerances

Update the controller constraints in your MoveIt configuration:

```yaml
constraints:
  goal_time: 0.5
  stopped_velocity_tolerance: 0.7
  joint1: {trajectory: 0.1, goal: 1}
  joint2: {trajectory: 0.1, goal: 1}
  joint3: {trajectory: 0.1, goal: 1}
  joint4: {trajectory: 0.1, goal: 1}
  joint5: {trajectory: 0.1, goal: 1}
```

### Solution 2: Increase planning time

Modify planning time parameters in your MoveIt configuration:

```yaml
planning_attempts: 10
planning_time: 5.0
```

### Solution 3: Try different planners

Experiment with different motion planners:

```bash
roslaunch move demo_gazebo.launch pipeline:=ompl
# or
roslaunch move demo_gazebo.launch pipeline:=chomp
```

## Controller-Related Issues

### Symptom: Joints oscillate or don't follow trajectories accurately

> [!TIP]
> Controller tuning is essential for smooth and accurate movements.

### Solution 1: Tune PID parameters

Adjust PID values in your controller configuration:

```yaml
gains:
  joint1:
    p: 100
    d: 1
    i: 1
    i_clamp: 1
```

Try increasing the P value for more responsive control or the D value to reduce oscillations.

### Solution 2: Check controller configuration

Ensure your controllers are properly configured in the `ros_controllers.yaml` file:

```yaml
arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
```

### Solution 3: Verify controller spawning

Make sure controllers are properly spawned in the launch files:

```bash
rosservice call /controller_manager/list_controllers
```

## Build and Package Problems

### Symptom: Build errors or runtime package not found errors

> [!CAUTION]
> Missing dependencies or build issues can prevent proper execution.

### Solution 1: Install missing dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Solution 2: Clean and rebuild

```bash
cd ~/projects/ROS_project
rm -rf build devel
catkin build
```

### Solution 3: Check package paths

Verify your package is in the ROS package path:

```bash
echo $ROS_PACKAGE_PATH
source devel/setup.bash
```

## Debugging Commands

Here are some useful commands for debugging your robot:

```bash
# Check robot state
rostopic echo /joint_states

# Monitor controller status
rostopic echo /arm_controller/state

# View available controllers
rosservice call /controller_manager/list_controllers

# Check MoveIt planning scene
rosrun tf view_frames

# Record simulation video
rosbag record -O robot_sim.bag /joint_states /arm_controller/state /tf
```

## Common Error Messages and Solutions

### Error: "No IK solver available for this group"

**Solution**: Make sure kinematics.yaml is correctly configured:

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

### Error: "Controller failed during execution"

**Solution**: Check controller constraints and make sure they're not too strict. Update the `stopped_velocity_tolerance` value.

### Error: "Controller handle ... could not be created"

**Solution**: Verify the controller name matches exactly between launch files and configuration files.

> [!TIP]
> If all else fails, you can use the ROS visualization tools to inspect the robot state and identify issues:
>
> ```bash
> rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
> ```
