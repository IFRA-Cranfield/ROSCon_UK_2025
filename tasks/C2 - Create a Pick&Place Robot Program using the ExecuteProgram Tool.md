# PART C: Integration of External Tools within the ROS 2-based Robotic System

## C2: Create a Pick&Place Robot Program using the ExecuteProgram Tool

In this task, you will learn how to create and execute a static sequence of Robot Movements using the ros2srrc_execution/ExecuteProgram tool.

__ExecuteProgram Tool__

ExecuteProgram.py is a Python script designed to automate the execution of static robotic programs within our ROS 2 environment. It is responsible for interpreting and executing predefined sequences of robot movements or actions stored in a .yaml file located within a specific ROS 2 package. These programs, written in YAML format, define a series of sequential steps to control robotic joints, grippers, or any other components involved in a given task.

Upon invocation, the script reads the specified program file (e.g., PROGRAM_NAME.yaml), which must be located in the /programs folder of any ROS 2 package. The following command must be used to execute a Robot Program:

```sh
ros2 run ros2srrc_execution ExecuteProgram.py package:="PACKAGE-NAME" program:="PROGRAM-NAME"
```

- _PACKAGE-NAME_: The name of the ROS 2 Package where your program is located (inside the /programs folder).
- _PROGRAM-NAME_: The name of the program you want to execute (without the .yaml extension).

__TASK__

In this exercise, you will:

- Execute the UR3-Demo program, to understand how Robot Programs are executed with the ExecuteProgram.py tool.
- Fill in the Robot Movement waypoints in the cubePP_ur3_sim.yaml and cubePP_ur3.yaml files, and execute the programs to perform the Cube Pick&Place task. The exact waypoint values (required robot poses) are included below.

</br>

_Execution Steps (UR3-Demo Program)_

1. Launch the ROSConUK25 UR3 Gazebo + MoveIt!2 Environment:
    ```sh
    ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_1  # UR3 Robot alone.
    ```

2. Execute the UR3-Demo Program:
    ```sh
    ros2 run ros2srrc_execution ExecuteProgram.py package:=rosconuk25_execution program:=ur3_demo
    ```

3. You can now close the ROS 2 Environment.

</br>

_Execution Steps (CubePP Program)_

1. Fill in the .yaml files with the ROBOT POSE values defined below. After doing the changes in the Robot Program files, the ROS 2 Workspace must be compiled in order to save the changes --> In a new terminal shell, COLCON BUILD:
    ```sh
    cd ~/dev_ws
    colcon build
    ```

2. Launch the ROSConUK25 UR3 Gazebo + MoveIt!2 Environment:
    ```sh
    ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_2  # UR3 + Robotiq HandE Gripper.
    ```

3. Spawn the WhiteCube to the Simulation Environment, to the following location:
    ```sh
    ros2 run ros2srrc_execution SpawnObject.py --package "rosconuk25_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x -0.2076 --y 0.1903 --z 0.92
    ```

4. Execute the CubePP Robot Program:
    ```sh
    ros2 run ros2srrc_execution ExecuteProgram.py package:=rosconuk25_execution program:=cubePP_ur3_sim
    ```

5. You can now close the ROS 2 Environment.

</br>

__Waypoints for Cube Pick & Place__

Pick-Approach Pose:
- x = -0.2076
- y = 0.1903
- z = 0.925
- qx = 0.0
- qy = 1.0
- qz = 0.0
- qw = 0.0

Pick Pose:
- x = -0.2076
- y = 0.1903
- z = 0.8625
- qx = 0.0
- qy = 1.0
- qz = 0.0
- qw = 0.0

Place-Approach Pose:
- x = 0.2082
- y = 0.1764
- z = 0.975
- qx = 0.0
- qy = 1.0
- qz = 0.0
- qw = 0.0

Place Pose:
- x = 0.2082
- y = 0.1764
- z = 0.925
- qx = 0.0
- qy = 1.0
- qz = 0.0
- qw = 0.0

</br>
</br>

---

For more information, please feel free to access:
- https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/ProgramExecution.md for more detailed instructions on how Robot Programs are executed using the __ExecuteProgram__ tool.