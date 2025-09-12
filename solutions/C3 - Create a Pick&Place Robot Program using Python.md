# PART C: Integration of External Tools within the ROS 2-based Robotic System

## C3: Create a Pick&Place Robot Program using Python

__COMMAND__

```sh
ros2 run rosconuk25_execution cubePP.py environment:="---"
```

__SOLUTION__

```sh
# 1. Launch the simulation environment:
ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_2

# 2. Spawn a cube within the environment:
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x -0.2076 --y 0.1903 --z 0.92

# 3. Execute the program:
ros2 run rosconuk25_execution cubePP.py environment:="gazebo"
```

In the real robot cell:

```sh
# 1. Launch the ROS 2 environment:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=rosconuk25 config:=ur3_2 robot_ip:=192.168.1.10

# 2. Place the cube on top of the robot workspace.

# 3. Execute the program:
ros2 run rosconuk25_execution cubePP.py environment:=robot
```