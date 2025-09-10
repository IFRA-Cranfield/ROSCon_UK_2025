# PART A: Introduction to ros2srrc ROS 2 Simulation and Control Packages

## A3: Launch Gazebo Simulation + MoveIt!2 Environment

In this exercise, the objective is to familiarise with the __ROS 2 Environment Launch procedure__ for the Robot Bringup + MoveIt!2 Framework environment. In ros2_SimRealRobotControl (ros2srrc), the ROS 2 Driver + MoveIt!2 Environment is launched with the following command:

```sh
ros2 launch ros2srrc_launch bringup_abb.launch.py package:=PACKAGE_NAME config:=CONFIG_NAME robot_ip:=IP_ADDRESS # For the ABB Robots.
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=PACKAGE_NAME config:=CONFIG_NAME robot_ip:=IP_ADDRESS # For the UR Robots.
```

</br>

In the workshop, this launch procedure will be tested with a UR3 robot:

```sh
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ros2srrc_ur3 config:=ur3_3 robot_ip:=192.168.1.10 # For the standard UR3 Robot.
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ur3cranfield config:=ur3cranfield_2 robot_ip:=192.168.1.10 # For the CranfieldUni UR3 Robot Cell.
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
- https://github.com/IFRA-Cranfield/ur3_CranfieldRobotics for more detail about how the UR3 robot is controlled using ROS 2.
- https://github.com/IFRA-Cranfield/irb120_CranfieldRobotics for more detail about how the ABB IRB-120 robot is controlled using ROS 2.
