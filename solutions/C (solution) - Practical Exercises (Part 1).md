TBD

(C1)

--

(C2)

ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_2
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x -0.2076 --y 0.1903 --z 0.92
ros2 run ros2srrc_execution ExecuteProgram.py package:=rosconuk25_execution program:=CubePP_ur3_sim

(C3)

ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_2
ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x -0.2076 --y 0.1903 --z 0.92
ros2 run rosconuk25_execution cubePP.py environment:=gazebo
ros2 run rosconuk25_execution cubePP.py environment:=robot