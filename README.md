# UR5e Arm MoveIt2 (ROS 2 Humble)

This repository provides a complete MoveIt2 configuration for the Universal Robots UR5e manipulator using ROS 2 Humble, together with a custom C++ execution node for motion planning and trajectory execution.

The current scope of the project is limited to the UR5e arm. Gripper integration is planned but not included in this repository.

---

## 1. Project Overview

The objective of this project is to configure and validate a fully operational motion planning pipeline for the UR5e robotic arm using:

- ROS 2 Humble
- MoveIt2
- OMPL motion planning
- KDL kinematics
- ur_robot_driver
- ros2_control

In addition to the standard MoveIt configuration, a custom C++ node is provided to demonstrate programmatic control of the manipulator through the MoveGroupInterface API.

The system supports:

- Joint-space motion planning
- Cartesian target planning
- Trajectory execution through FollowJointTrajectory
- A predefined motion sequence including Cartesian perturbations ("shake" motion)

---

## 2. Repository Structure

```
ur5e-arm-moveit2/
├── ur5e_arm_description/
├── ur5e_arm_moveit_config/
└── ur5e_contactile_actions/
```

### 2.1 ur5e_arm_description

Contains the URDF model of the UR5e manipulator.

The model defines:

- World reference frame
- Base link
- Six revolute joints
- Tool frame (`tool0`)
- Mesh references from `ur_description`

This package provides the robot model used by both `robot_state_publisher` and MoveIt2.

---

### 2.2 ur5e_arm_moveit_config

Provides the MoveIt2 configuration required for motion planning.

Includes:

- SRDF definition (planning groups and kinematic chain)
- OMPL planning pipeline configuration
- KDL kinematics configuration
- Joint limits
- Controller configuration
- Launch file for driver + MoveIt + RViz bringup

Key configuration elements:

- Planning group: `ur_manipulator`
- End-effector link: `tool0`
- Planner: RRTConnect
- Controller: `scaled_joint_trajectory_controller` (FollowJointTrajectory)

---

### 2.3 ur5e_contactile_actions

Custom C++ executable implementing high-level motion logic.

The node:

- Connects to MoveIt via MoveGroupInterface
- Loads robot_description and semantic parameters
- Plans and executes joint and Cartesian trajectories
- Implements a predefined motion sequence:
  1. Move to home configuration
  2. Move to grasp joint configuration
  3. Move to Cartesian pose offset
  4. Execute alternating wrist rotation ("shake")
  5. Return to home configuration

Gripper-related hooks exist in the codebase but are currently disabled at compile time.

---

## 3. System Architecture

The execution pipeline is structured as follows:

```
UR5e Hardware
      │
      ▼
ur_robot_driver (ros2_control)
      │
      ▼
MoveIt2 (move_group)
      │
      ▼
ur5e_contactile_actions_node
```

The `ur_robot_driver` provides hardware abstraction through ros2_control.  
MoveIt2 performs planning and trajectory generation.  
The custom C++ node acts as a high-level task controller.

---

## 4. Requirements

Software:

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt2
- ur_robot_driver
- ur_description
- kdl_kinematics_plugin
- C++17 compatible compiler

Hardware:

- Universal Robots UR5e
- Network access to robot controller

---

## 5. Build Instructions

From the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To build only this project:

```bash
colcon build --packages-select \
  ur5e_arm_description \
  ur5e_arm_moveit_config \
  ur5e_contactile_actions
```

---

## 6. Execution

### 6.1 Bringup

Launch the UR driver and MoveIt:

```bash
ros2 launch ur5e_arm_moveit_config ur5e_arm_moveit_bringup.launch.py
```

This will:

- Start `ur_robot_driver`
- Start MoveIt `move_group`
- Launch RViz

Ensure that:

- The robot is in remote control mode
- The IP address in the launch file matches the robot configuration

---

### 6.2 Run Motion Execution Node

In a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ur5e_contactile_actions ur5e_contactile_actions_node
```

---

## 7. Motion Planning Configuration

Planning:

- Planning pipeline: OMPL
- Planner: RRTConnect
- Planning group: `ur_manipulator`
- End-effector link: `tool0`
- Reference frame: `base_link`

Kinematics:

- Plugin: KDLKinematicsPlugin

Execution:

- Controller type: FollowJointTrajectory
- Controller name: `scaled_joint_trajectory_controller`

---

## 8. Execution Node Parameters

The node exposes configurable parameters:

| Parameter | Description |
|------------|-------------|
| planning_group | MoveIt planning group |
| eef_link | End-effector link |
| ref_frame | Reference frame |
| home_joint_values | Joint configuration for home |
| grasp_joint_values | Joint configuration for grasp |
| shake_angle_delta | Shake amplitude (radians) |
| num_shakes | Number of shake cycles |
| enable_gripper | Reserved for future use |

Example:

```bash
ros2 run ur5e_contactile_actions ur5e_contactile_actions_node --ros-args \
  -p num_shakes:=4
```

---

## 9. Known Limitations

- Gripper is not integrated
- No combined arm + gripper planning group
- No dynamic obstacle insertion
- No simulation-only configuration provided
- RViz configuration is not customized
