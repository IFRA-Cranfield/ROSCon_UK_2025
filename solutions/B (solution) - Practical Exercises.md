TBD

ros2 launch ros2srrc_launch simulation.launch.py package:=rosconuk25 config:=ur3_1
ros2 launch ros2srrc_launch simulation.launch.py package:=rosconuk25 config:=ur3_2

ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_1
ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_2

ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "BlackCube.urdf" --name "BlackCube" --x -0.2076 --y 0.1903 --z 0.92