# Technical Documentation for Robot Arm Project

## Code Architecture

The project uses a Python-based control system built on ROS and MoveIt to operate a 4-DOF robotic arm with a gripper. Below is a technical breakdown of the implementation.

## Key Files and Their Functions

### 1. `code.py`

This is the main execution script for controlling the robot arm. It defines a `MyRobot` class that encapsulates the functionality to control different robot groups (arm and hand) through MoveIt.

#### Class Structure

#### **MyRobot Class**

```python
class MyRobot:
    def __init__(self, Group_Name):
        # Initialize ROS, MoveIt, and set up interfaces to the robot

    def set_pose(self, arg_pose_name):
        # Move the robot to a predefined pose

    def __del__(self):
        # Clean up when the object is destroyed
```

#### Key Methods

1. **`__init__(self, Group_Name)`**

   - Initializes MoveIt Commander
   - Creates interfaces to robot, planning scene, and move group
   - Sets up action client for trajectory execution
   - Retrieves and displays planning frame, end effector link, and group names

2. **`set_pose(self, arg_pose_name)`**

   - Sets a named target pose for the move group
   - Plans a trajectory to the target pose
   - Executes the planned trajectory

3. **`__del__(self)`**
   - Cleans up by shutting down MoveIt Commander

#### Main Execution

The `main()` function:

1. Creates instances of `MyRobot` for the arm and hand groups
2. Opens the gripper initially
3. Enters a loop that performs the pick and place sequence:
   - Move to zero position
   - Move to pick position
   - Close the gripper
   - Lift the object
   - Turn to drop location
   - Move to drop position
   - Open the gripper
4. The loop continues until ROS is shutdown

## URDF Model Structure

The robot is defined in the URDF file (`arm_assem_urdf_v3.urdf`), which specifies:

- Links (rigid bodies)
- Joints (connections between links)
- Visual elements (for display)
- Collision elements (for physics)
- Gazebo-specific properties

### Links Structure

```bash
world
└── base_link
    └── Link1
        └── link2
            └── link3
                └── link4
                    └── link5
                        ├── rightGripper
                        │   └── r_finger
                        └── leftGripper
                            └── l_fing
```

### Collision Configuration

The simulation uses carefully defined collision properties to prevent self-collision while maintaining realistic physics:

1. **Self-Collision Settings**:

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

2. **Collision Pair Exclusions**:

   ```xml
   <disable_collision>rightGripper leftGripper</disable_collision>
   <disable_collision>rightGripper l_fing</disable_collision>
   <disable_collision>leftGripper r_finger</disable_collision>
   <disable_collision>r_finger l_fing</disable_collision>
   ```

## MoveIt Configuration

MoveIt is configured using several key files:

1. **arm_assem_urdf_v3.srdf**: Defines move groups, joint states, end effectors, and collision exclusions
2. **ros_controllers.yaml**: Configures controllers for joints
3. **joint_limits.yaml**: Sets limits for joint movement
4. **kinematics.yaml**: Specifies kinematics solver parameters

### Controller Configuration

The arm uses trajectory controllers configured as follows:

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

## Launch Files System

The project uses a hierarchical launch system:

1. **full_robot_arm_sim.launch**: Main entry point that launches:
   - arm_urdf.launch: Loads the robot model
   - move_group.launch: Starts the MoveIt move group
   - moveit_rviz.launch: Opens RViz with MoveIt configuration

## Technical Challenges and Solutions

### 1. Collision Issues

**Problem**: Gripper fingers would collide with each other or with the gripper base.

**Solution**:

- Set `selfCollide` to false for gripper components
- Add explicit `disable_collision` tags between potentially colliding parts
- Fine-tune collision mesh geometry

### 2. Velocity Tolerance Issues

**Problem**: Robot would fail to reach target positions due to strict velocity tolerances.

**Solution**:

- Increased `stopped_velocity_tolerance` to 0.7
- Added per-joint trajectory and goal tolerances
- Set appropriate goal time constraints

### 3. Controller Tuning

**Problem**: Oscillations during movement due to inappropriate controller parameters.

**Solution**:

- Tuned PID values in controller configuration
- Set appropriate gains for each joint
- Added constraints to limit acceleration

## Algorithm Details

### Pick and Place Sequence

The pick and place operation follows this algorithm:

1. **Initialization**:

   ```python
   arm = MyRobot("arm")
   hand = MyRobot("hand")
   hand.set_pose("zero")
   ```

2. **Pick Operation**:

   ```python
   arm.set_pose("zero")
   arm.set_pose("pick")
   hand.set_pose("close")
   ```

3. **Transport Operation**:

   ```python
   arm.set_pose("lift")
   arm.set_pose("turn")
   ```

4. **Place Operation**:

   ```python
   arm.set_pose("drop")
   hand.set_pose("open")
   ```

### Motion Planning

The motion planning process happens in these steps:

1. Set named target pose using `set_named_target`
2. Generate plan using `plan()`
3. Execute trajectory via action server
4. Wait for completion and log results

## Development and Testing Tips

1. **Simulation Testing**:

   - Test movements in small increments
   - Monitor joint states and controller status
   - Use RViz to visualize planned paths before executing

2. **Common Debug Commands**:

   ```bash
   # Check controller status
   rostopic echo /arm_controller/state

   # Monitor joint states
   rostopic echo /joint_states

   # Check controller parameters
   rosparam get /arm_controller
   ```

3. **Performance Optimization**:
   - Reduce mesh complexity for faster physics calculations
   - Use simplified collision meshes
   - Adjust physics engine parameters for better stability
