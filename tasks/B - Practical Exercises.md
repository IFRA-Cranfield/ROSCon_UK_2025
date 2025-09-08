# Exercise B1: Create a ROS 2 Environment for the UR3 Robot

This exercise focuses on generating the **URDF/Xacro** for a specific UR3 robot cell and wiring it into the standard `ros2_SimRealRobotControl` (ros2srrc) environment. Two user-facing packages are provided and **intentionally empty** so you can build them up:
- `rosconuk25_gazebo`
- `rosconuk25_moveit2`

You will:

1. Define robot **Configurations** in `rosconuk25_gazebo/config/configurations.yaml`.
2. Create three **URDF/Xacro** files (focus on 1 and 2; 3 is provided and only for reference).
3. (Provided) Inspect the **MoveIt!2** configs (`.srdf`, `.rviz`) and the gazebo/world file.

> **Focus:** Implement configuration **1** and **2** completely. Configuration **3** (cube pose estimation use-case) is provided and can be ignored during B1.

## Step 1 — Define Robot Configurations

Edit (or create) the file: `rosconuk25_gazebo/config/configurations.yaml` and add entries for the three setups. Use the template below and **fill the TODOs** for URDF filename and the `rob` / `ee` keys.

```yaml
# UR3 - Cranfield University:
Configurations:

  - ID: "ur3_1"
    Name: "UR3 Robot alone on Cranfield University (IA Lab) Stand."
    urdf: "<FILL>.urdf.xacro"        # TODO: set file name created in Step 2 (case 1)
    rob: "ur3"                       # TODO: robot name used by ros2srrc_robots
    ee: ""                           # TODO: leave empty (no end-effector)

  - ID: "ur3_2"
    Name: "UR3 Robot + Robotiq HandE Gripper on Cranfield University (IA Lab) Stand."
    urdf: "<FILL>.urdf.xacro"        # TODO: set file name created in Step 2 (case 2)
    rob: "ur3"                       # TODO: robot name used by ros2srrc_robots
    ee: "robotiq_hande"              # TODO: end-effector name used by ros2srrc_endeffectors

  - ID: "ur3_3"
    Name: "UR3 Robot + Robotiq HandE Gripper on Cranfield University (IA Lab) Stand: Cube Pose Estimation use-case."
    urdf: "ur3_rosconuk25_3.urdf.xacro"
    rob: "ur3"
    ee: "robotiq_hande"
```

## Step 2 — Create the URDF/Xacro files (focus on cases 1 and 2)

Create three files under your package (e.g., `rosconuk25_gazebo/urdf/`):

- `ur3_rosconuk25_1.urdf.xacro` (**no end-effector**)
- `ur3_rosconuk25_2.urdf.xacro` (**with Robotiq HandE**)
- `ur3_rosconuk25_3.urdf.xacro` (**provided; reference only**)

A starting template `empty.urdf.xacro` is provided. Copy it and **fill the three areas** below:

1. Include the generic UR3 and end-effector macro files (from `ros2srrc_robots` and `ros2srrc_endeffectors`).
2. Fill CAD mesh references (provided in your `meshes/` folder).
3. Set joint parents/children (`world` ↔ stand, stand ↔ `base_link`, etc.).

## Step 3 — Inspect the provided world and MoveIt!2 configs

These are **provided** for you:

- **World file** (Gazebo).
- **MoveIt!2** configuration files (`.srdf`, `.rviz`, etc.).

> **Encouraged:** Open them to understand how the planning groups, collision geometry, and RViz layout relate to your URDF links/joints.

## Validate your environment

After Step 1–2, you should be able to launch your setups. Launch the ROS 2 Environments using the commands below (same as in Task A), and add the reference to the generated package and configurations.

- **Gazebo Simulation**  

  ```bash
  ros2 launch ros2srrc_launch simulation.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
  ```

- **Gazebo + MoveIt!2**
  
  ```bash
  ros2 launch ros2srrc_launch moveit2.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
  ```

### Quick checks

- Robot appears at the stand; links and joints form a single connected tree.
- No missing mesh warnings in terminal (fix mesh paths if needed).
- For case 2, the end-effector is attached to the correct tool link.
- Controllers load in Gazebo without errors.
- RViZ launches and you are able to move the robot.

</br>
</br>

---
---

</br>
</br>

# Exercise B2: Add Objects to the ROS 2 Simulation Environment 

In this exercise you will add **objects** (cubes) to your robot environment, preparing for the pick-and-place task in Exercise C.

You will:

1. Identify cube CAD files in `meshes/objects/`.
2. Create cube **URDF/Xacro** files in `urdf/objects/`.
3. Spawn the cubes into Gazebo using `ros2_SimRealRobotControl` utilities.

## Step 1 — Identify CAD meshes

- Identify the provided CAD files of the cubes in: `rosconuk25_gazebo/meshes/objects/`
- Format -> .dae (collada, mesh).

## Step 2 — Create object URDF/Xacro files

1. Create a URDF file per object in: `rosconuk25_gazebo/urdf/objects/`
2. Use the template provided and fill in the mesh filename (CAD reference).

## Step 3 — Spawn objects into Gazebo

Use the ros2srrc execution tool to spawn objects. The command pattern is:

```bash
ros2 run ros2srrc_execution SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}
```

## Validate your objects

- Cubes appear at the requested coordinates and rest stably.
- You can spawn multiple instances by changing `--name` and positions.