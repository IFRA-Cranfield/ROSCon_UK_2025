# EXERCISE C (PART 2) - Solutions

## C4 - Perception‑Driven Pick & Place

### Completed files

In rosconuk25_ope:
- python/cubePP_detection.py script.

### Solution

In simulation:

```sh
# 1. Launch the simulation environment:
ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_3

# 2. Spawn a cube within the environment:
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x -0.13 --y 0.2 --z 0.92

# 3. Move the Robot to the SidePose to make sure the ArUco Grid is visible:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 180.00, joint2: -90.00, joint3: 90.00, joint4: -90.00, joint5: -90.00, joint6: 0.00}, speed: 1.0}"

# 4. Execute the Cube Detection and Pose Estimation script:
ros2 run rosconuk25_ope PositionEstimation.py environment:=gazebo model:=ColouredCubes_ur3 visualize:=true

# 5. Execute the program:
ros2 run rosconuk25_ope cubePP_detection.py environment:=gazebo cube:=WhiteCube
```

In the real robot cell:

```sh
# 1. Launch the simulation environment:
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=rosconuk25 config:=ur3_3 robot_ip:=192.168.1.10

# 2. Place the cube on top of the robot workspace.

# 3. Move the Robot to the SidePose to make sure the ArUco Grid is visible:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 180.00, joint2: -90.00, joint3: 90.00, joint4: -90.00, joint5: -90.00, joint6: 0.00}, speed: 1.0}"

# 4. Execute the Cube Detection and Pose Estimation script:
ros2 run rosconuk25_ope PositionEstimation.py environment:=robot model:=ColouredCubes_ur3 visualize:=true

# 5. Execute the program:
ros2 run rosconuk25_ope cubePP_detection.py environment:=robot cube:=WhiteCube
```