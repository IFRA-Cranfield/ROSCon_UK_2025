# EXERCISE C (PART 1) - Solutions

## C1 - Operate the Robot with ROS 2 Actions and Monitor the Robot Pose using a ROS 2 Topic

### Reference

https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/RobotOperation.md

### Solution

Firstly, launch the ROS 2 Environment (w/ MoveIt!2):

```sh
# In simulation:
ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_1

# For the real robot:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=rosconuk25 config:=ur3_1 robot_ip:=192.168.1.10
```

(NOTE): Feel free to test with any other Robot/End-Effector. But please, remember to launch the moveit2.launch file, which contains the ROS 2 Servers for the mentioned tools!

Test the ROS 2 Tools:

```sh
# MoveJ:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}"
# MoveL:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.00}, speed: 1.0}"
# MOveR:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveR', mover: {joint: '--', value: 0.00}, speed: 1.0}"
# MoveROT:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
# MoveRP:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.00, y: 0.00, z: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
# MoveG (for the gripper):
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveG', moveg: 0.0, speed: 1.0}"

# RobMove:
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: '---', speed: 1.0, x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0}"
```

```sh
# To check the state of the joints:
ros2 run ros2srrc_execution RobotState.py
ros 2 topic echo /joint_states

# To check the end-effector pose:
ros2 topic echo /Robpose
```

## C2 - Execute a Pick & Place Robot Program (.yaml)

### Reference

https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/ProgramExecution.md  

### Completed files

In rosconuk25_execution:
- programs/cubePP_ur3_sim.yaml -> Simulation
- programs/cubePP_ur3_.yaml -> Real Robot

### Command

```sh
ros2 run ros2srrc_execution ExecuteProgram.py package:="PACKAGE_NAME" program:="PROGRAM_NAME"
```

### Solution

In simulation:

```sh
# 1. Launch the simulation environment:
ros2 launch ros2srrc_launch simulation.launch.py package:=rosconuk25 config:=ur3_2

# 2. Spawn a cube within the environment:
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x -0.2076 --y 0.1903 --z 0.92

# 3. Execute the program:
ros2 run ros2srrc_execution ExecuteProgram.py package:=rosconuk25_execution program:=CubePP_ur3_sim
```

In the real robot cell:

```sh
# 1. Launch the ROS 2 environment:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=rosconuk25 config:=ur3_2 robot_ip:=192.168.1.10

# 2. Place the cube on top of the robot workspace.

# 3. Execute the program:
ros2 run ros2srrc_execution ExecuteProgram.py package:=rosconuk25_execution program:=CubePP_ur3
```

## C3 - Implement the Same Pick & Place with a Python Script

### Reference

https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/ros2srrc_execution/python/testClients

### Completed files

In rosconuk25_execution:
- python/cubePP.py script.

### Command

```sh
ros2 run rosconuk25_execution cubePP.py environment:="---"
```

### Solution

In simulation:

```sh
# 1. Launch the simulation environment:
ros2 launch ros2srrc_launch simulation.launch.py package:=rosconuk25 config:=ur3_2

# 2. Spawn a cube within the environment:
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x -0.2076 --y 0.1903 --z 0.92

# 3. Execute the program:
ros2 run rosconuk25_execution cubePP.py environment:="gazebo
```

In the real robot cell:

```sh
# 1. Launch the ROS 2 environment:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=rosconuk25 config:=ur3_2 robot_ip:=192.168.1.10

# 2. Place the cube on top of the robot workspace.

# 3. Execute the program:
ros2 run rosconuk25_execution cubePP.py environment:=robot
```