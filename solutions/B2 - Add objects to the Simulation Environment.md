# PART B: Setting up your own Robot Cell using IFRA's ROS 2 Packages

## B2: Add objects to the Simulation Environment

__COMMAND__

```sh
ros2 run ros2srrc_execution SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}
```

__SOLUTION__

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