# PART A: Introduction to ros2srrc ROS 2 Simulation and Control Packages

## A2: Launch Gazebo Simulation + MoveIt!2 Environment

__COMMAND__

```sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

__SOLUTION__

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

(NOTE): Some robots (not all of them) have been included.