# PART A: Introduction to ros2srrc ROS 2 Simulation and Control Packages

## A2: Launch Gazebo Simulation + MoveIt!2 Environment

In this exercise, the objective is to familiarise with the __ROS 2 Environment Launch procedure__ for the Gazebo Simulation + MoveIt!2 Framework environment. In ros2_SimRealRobotControl (ros2srrc), the Gazebo+MoveIt!2 Environment is launched with the following command:

```sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=PACKAGE_NAME config:=CONFIG_NAME
```

</br>

Feel free to test the following PACKAGE/CONFIG combinations available in ros2_SimRealRobotControl:

Package: ros2srrc_irb120
Config: 
- irb120_1: ABB IRB-120 on top of Robot Stand.
- irb120_2: ABB IRB-120 + Schunk EGP-64 Gripper on top of Robot Stand.
- irb120_21: ABB IRB-120 + Schunk EGP-64 Gripper (rounded fingers) on top of Robot Stand.
- irb120_3: ABB IRB-120 + Lamination Sheet Vacuum-Gripper on top of Robot Stand.

Package: ros2srrc_irb1600
Config:
- irb1600_1: ABB IRB-1600 on top of Robot Stand.

Package: ros2srrc_ur3
Config:
- ur3_1: UR3 on top of Robot Stand.
- ur3_2: UR3 + Robotiq 2f-85 gripper on top of Robot Stand.
- ur3_3: UR3 + Robotiq HandE gripper on top of Robot Stand.

Package: ros2srrc_ur5e
Config:
- ur5e_1: UR5e on top of Robot Stand.
- ur5e_2: UR5e + Robotiq 2f-85 gripper on top of Robot Stand.
- ur5e_3: UR5e + Robotiq HandE gripper on top of Robot Stand.

Package: ros2srrc_iiwa
Config:
- iiwa_1: KUKA LBR-iiwa on top of Robot Stand.
- iiwa_2: KUKA LBR-iiwa + Robotiq 2f-85 gripper on top of Robot Stand.

Package: ros2srrc_rx160
Config:
- rx160_1: Staubli RX-160 on top of Robot Stand.

</br>

EXAMPLE -> UR5e + Robotiq Hand-E gripper:

```sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5e config:=ur5e_3
```

</br>
</br>

---

__RVIZ-based Robot Control__

Once the environment is launched, you will be able to control the robot using RViZ's interactive HMI. Follow these steps:

- Under the "PLANNING" tab, make sure that the Robot Arms planning group is selected (if the launched robot has an end-effector, the EE could be selected by default).
- Under the "CONTEXT" tab, select the MOVEMENT TYPE: PTP for Joint-Space movement, LIN for Cartesian-Space movement.
- Move the robot using the interactive markers, and, under the "PLANNING" tab, click -> PLAN & EXECUTE!

</br>
</br>

---

For more information, please feel free to access:
- https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/ROS2EnvironmentLaunch.md for better detail about the ROS 2 Environment Launch process.
- https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/packages to check all available standard ros2srrc robot + end-effector configurations.