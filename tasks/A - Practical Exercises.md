# Exercise A: Launching ROS 2 Environments and Controlling Robot Arms in Simulation and Real Hardware

This exercise covers launching Gazebo simulation environments, adding MoveIt!2 for motion planning, and connecting to a physical robot arm.

## Before you start...

- Review the ROS 2 environment launch guide: [ROS2EnvironmentLaunch.md](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/ROS2EnvironmentLaunch.md)
- Browse available robots, end-effectors, and layouts: [ros2_SimRealRobotControl/packages](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/packages)

## Exercise A1: Launch Gazebo Simulation

### Goal

Start the Gazebo environment with a chosen robot and end-effector from `ros2_SimRealRobotControl`.

### Command

```bash
ros2 launch ros2srrc_launch simulation.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

### Parameters

- `<PACKAGE_NAME>`: robot/end-effector package (see the `packages/` list).
- `<CONFIG_NAME>`: configuration/layout for that package (see the package_gazebo/config/configurations.yaml file for reference).

### Try

- Observe the robot in Gazebo.
- Identify the attached end-effector.

### Questions

- What changes when you switch `<CONFIG_NAME>`?
- Can you find a configuration with both a robot arm and an end-effector?

## Exercise A2: Add MoveIt!2 and Control in RViz

### Goal

Control the robot in simulation using MoveIt!2.

### Command

```bash
ros2 launch ros2srrc_launch moveit2.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

### Notes

- RViz opens with the MoveIt!2 interface.
- Use the interactive markers to plan and execute motions.

### Try

- Plan and execute an end-effector motion.
- Plan and execute a robot arm motion.

### Questions

- What’s the difference between A1 (Gazebo only) and A2 (Gazebo + MoveIt!2)?
- Which component performs planning and collision checking?

---

## Exercise A3: Connect to the Real UR3 Robot

### Goal

Launch the ROS 2 Driver and control the physical UR3 robot using MoveIt!2.

### Command

```bash
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME> robot_ip:=<ROBOT_IP>
```

### Parameters

- `<ROBOT_IP>`: IP address of the physical UR3 robot.

### Notes

- Use RViz (MoveIt!2) to send commands to the real robot.
- **Safety:** Ensure the workspace is clear before executing motions.

### Try

- Move the end-effector in Cartesian space.
- Compare real robot motion vs. simulation.

### Questions

- How does controlling hardware differ from simulation?
- What issues commonly appear when moving from sim to real (e.g., calibration, network latency, safety stops)?