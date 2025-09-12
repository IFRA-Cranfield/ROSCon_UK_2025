# PART A: Introduction to ros2srrc ROS 2 Simulation and Control Packages

## A3: Launch Gazebo Simulation + MoveIt!2 Environment

__COMMAND__

```sh
ros2 launch ros2srrc_launch bringup_ROB.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME> robot_ip:=<IP_ADDRESS>
```

__SOLUTION__

```sh
# UR3 Robot:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ros2srrc_ur3 config:=ur3_1 robot_ip:=192.168.1.10

# UR3 Robot alone on Cranfield University (IA Lab) Stand:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ur3cranfield config:=ur3cranfield_1 robot_ip:=192.168.1.10
# UR3 Robot + Robotiq HandE Gripper on Cranfield University (IA Lab) Stand: CubeTray Pick&Place use-case.
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ur3cranfield config:=ur3cranfield_2 robot_ip:=192.168.1.10
```