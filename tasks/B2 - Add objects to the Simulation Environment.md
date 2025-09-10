# PART B: Setting up your own Robot Cell using IFRA's ROS 2 Packages

## B2: Add objects to the Simulation Environment

All the FIXED components of a Robot Cell (e.g. Robot Stand, Fixed Boxes, Cameras...) can be included inside the Robot's URDF. These components are fixed during the whole simulation process, and can be imported as physical entities and linked to the environment as fixed joints in the URDF.

However, this is not the case for all the moving/manipulated objects, such as the coloured cubes (objects used in this repository). These need to be represented as individual URDF files, and spawed into the Gazebo Simulation Environment using a dedicated ROS 2 Node. 

This exercise consists of 2 different tasks:

### 1. Understanding object's URDF file structure

Take some time understanding how:

- Object CAD files are included inside the /meshes folder.
- Object URDF files are included inside the /urdf/objects folder.

Please do take a look into the object.urdf files:

- How the physical parameters of the object are defined.
- How the CAD files are imported to the urdf.
- How the {name} variable is defined and used throughout the urdf.

### 2. Spawn objects to the ROSConUK25 UR3 Cell's Gazebo Simulation Environment

In ros2_SimRealRobotControl, we use the following command to spawn objects into the Gazebo Simulation Environment:

```bash
ros2 run ros2srrc_execution SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}
```

For this command to work properly, the URDF files of the objects to spawn have to be located inside the urdf/objects folder of a ROS 2 Package. Then:

- package: The name of the ROS 2 Package -> rosconuk25_gazebo
- urdf: The name of the URDF file -> BlueCube.urdf, GreenCube.urdf, WhiteCube.urdf, RedCube.urdf
- name: It can be any name you want. This enables having different instantiations of the same object in the Simulation.
- x, y, z: 3D-coordinates where the object is spawned.

__TASK__

1. Launch the ROS 2 Gazebo Simulation Environment of the ROSConUK25 UR3 Robot Cell (learned in B1).
2. Open a new Terminal Shell, and spawn any coloured cube on top of the UR3 Stand. 

    NOTE: The units are (m) and the origin is in the centre of the robot's base link, ground level.

</br>
</br>