# PART B: Setting up your own Robot Cell using IFRA's ROS 2 Packages

## B1: Set up a ROS 2 Environment for your Robot Cell

As explained in Part B in the workshop, the creation of a new ROS 2 Environment for a Robot Cell requires generating 2 new ROS 2 Packages:

- A Gazebo ROS 2 Package.
- A MoveIt!2 ROS 2 Package.

In ros2_SimRealRobotControl, the standard naming convention we use is:

- We give a name to the package set (Robot Cell name) -> rosconuk25
- The Gazebo package is named -> rosconuk25_gazebo
- The MoveIt!2 package is named -> rosconuk25_moveit2

This exercise consists of 4 different tasks:

### 1. Understanding the configurations.yaml file

Spend some time understanding the rosconuk25_gazebo/config/configurations.yaml file. This file contains the description of all different Robot Cell layout variations, and its links to the Robot Cell's URDF files.

### 2. Understanding the structure of the URDF files

Have a look into the Robot Cell's URDF files. Pay special attention to:

- How the ROBOT's and END-EFFECTOR's standard URDF's are imported from ros2srrc_robots and ros2srrc_endeffectors repositories.
- How the world/ur3_stand components are imported to the URDF, and how the robot is linked and positioned on top of the Robot Stand.
- How the CAD files are imported to the URDF.

### 3. MoveIt!2 Package

The _moveit2 package contains 2 different files, which are required by MoveIt!2 to configure the control of the Robot Cell:

- srdf file: The Semantic Robot Description Format (SRDF) file contains information about the robot's kinematic properties, self-collision matrices, and predefined poses (if any). This file complements the URDF and is required by MoveIt!2 to perform kinematic calculations and collision avoidance. 
- rviz file: This file defines the visual representation of the robot in RVIZ, the ROS 2 visualization tool. It specifies the robot model, display parameters, and any pre-configured markers or interactive controls. RVIZ is used to visualize the robot's state and motion planning.

A set of srdf+rviz file is required per every robot or robot+end-effector combination.

### 4. Launch the ROSConUK25 UR3 Robot Cell's ROS 2 Environment!

From PART A, we know that a Gazebo + MoveIt!2 ROS 2 Environment can be launched using the following command:

```sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=PACKAGE_NAME config:=CONFIG_NAME
```

- For our UR3 Robot Cell in this repository, which parameter values correspond to PACKAGE_NAME and CONFIG_NAME?
- How many Robot Cell layouts/variants do we have?

</br>
</br>

---

For more information, please feel free to access:
- https://github.com/IFRA-Cranfield/irb120_CranfieldRobotics for more detail about how the ROS 2 Gazebo + MoveIt!2 Packages were build for Cranfield University's ABB IRB-120 Robot Cell.
- https://github.com/IFRA-Cranfield/ur3_CranfieldRobotics for more detail about how the ROS 2 Gazebo + MoveIt!2 Packages were build for Cranfield University's UR3 Robot Cell.