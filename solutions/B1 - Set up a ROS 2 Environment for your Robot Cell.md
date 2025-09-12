# PART B: Setting up your own Robot Cell using IFRA's ROS 2 Packages

## B1: Set up a ROS 2 Environment for your Robot Cell

__COMMAND__

```sh
ros2 launch ros2srrc_launch simulation.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

__SOLUTION__

```sh
# Gazebo Simulation:
ros2 launch ros2srrc_launch simulation.launch.py package:=rosconuk25 config:=ur3_1
ros2 launch ros2srrc_launch simulation.launch.py package:=rosconuk25 config:=ur3_2

# Gazebo Simulation + MoveIt!2 Framework:
ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_1
ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_2

# Robot Bringup + MoveIt!2 Framework:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=rosconuk25 config:=ur3_1 robot_ip:=192.168.1.10
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=rosconuk25 config:=ur3_2 robot_ip:=192.168.1.10
```