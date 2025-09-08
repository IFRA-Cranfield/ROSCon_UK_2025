# EXERCISE B - Solutions

## B1 - Create a ROS 2 Environment for the UR3 Robot

### Completed files

In rosconuk25_gazebo:
- config/configurations.yaml 
- urdf/ur3_rosconuk25_1.urdf.xacro and ur3_rosconuk25_2.urdf.xacro

### Command

```sh
ros2 launch ros2srrc_launch simulation.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

### Solutions

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

## B2 - Add Objects to the ROS 2 Simulation Environment 

### Completed files

In rosconuk25_gazebo:
- urdf/objects -> URDF files for all coloured cubes.

### Command

```sh
ros2 run ros2srrc_execution SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}
```

### Solution

Launch the Gazebo Simulation environment:
```sh
ros2 launch ros2srrc_launch simulation.launch.py package:=rosconuk25 config:=ur3_1
```

After launching the simulation environment, you can spawn the objects within the Robot Cell:
```sh
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "BlueCube.urdf" --name "BlueCube" --x 0.0 --y 0.3 --z 1.0
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "GreenCube.urdf" --name "GreenCube" --x 0.0 --y 0.3 --z 1.0
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "RedCube.urdf" --name "RedCube" --x 0.0 --y 0.3 --z 1.0
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x 0.0 --y 0.3 --z 1.0
```