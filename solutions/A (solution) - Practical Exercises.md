# EXERCISE A - Solutions

### Reference

https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/ROS2EnvironmentLaunch.md

https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/packages

## A1 - Launch Gazebo Simulation

### Command

```sh
ros2 launch ros2srrc_launch simulation.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

### Solution

```sh
# ABB IRB-120:
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_irb120 config:=irb120_1
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_irb120 config:=irb120_2
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_irb120 config:=irb120_3

# ABB IRB-1200:
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_irb1200 config:=irb1200_1

# ABB IRB-1600:
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_irb1600 config:=irb1600_1

# UR3:
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur3 config:=ur3_1
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur3 config:=ur3_2
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur3 config:=ur3_3

# UR5:
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur5 config:=ur5_1
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur5 config:=ur5_2
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur5 config:=ur5_3

# UR10e:
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur10e config:=ur10e_1
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur10e config:=ur10e_2
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_ur10e config:=ur10e_3

# Kuka LBR-iiwa:
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_iiwa config:=iiwa_1
```

## A2 - Launch Gazebo Simulation + MoveIt!2 Framework

### Command

```sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

### Solution

```sh
# ABB IRB-120:
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_irb120 config:=irb120_1
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_irb120 config:=irb120_2
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_irb120 config:=irb120_3

# ABB IRB-1200:
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_irb1200 config:=irb1200_1

# ABB IRB-1600:
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_irb1600 config:=irb1600_1

# UR3:
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur3 config:=ur3_1
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur3 config:=ur3_2
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur3 config:=ur3_3

# UR5:
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_1
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_2
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_3

# UR10e:
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur10e config:=ur10e_1
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur10e config:=ur10e_2
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur10e config:=ur10e_3

# Kuka LBR-iiwa:
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_iiwa config:=iiwa_1
```

## A3 - Connect to Robot (ROS 2 Driver) + MoveIt!2 Framework

### Command

```sh
ros2 launch ros2srrc_launch bringup_ROB.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME> robot_ip:=<IP_ADDRESS>
```

### Solution

```sh
# UR3 Robot:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ros2srrc_ur3 config:=ur3_1 robot_ip:=192.168.1.10

# UR3 Robot alone on Cranfield University (IA Lab) Stand:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ur3cranfield config:=ur3cranfield_1 robot_ip:=192.168.1.10
# UR3 Robot + Robotiq HandE Gripper on Cranfield University (IA Lab) Stand: CubeTray Pick&Place use-case.
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ur3cranfield config:=ur3cranfield_2 robot_ip:=192.168.1.10
```