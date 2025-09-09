# Exercise C (PART 2): Perception‑Driven Pick & Place

## Introduction

In this task you will complete a **perception‑driven pick & place** with the UR3. The perception pipeline (already provided) detects coloured cubes and estimates their 3D pose. **Your job** is to write a Python script that **subscribes** to the published cube pose and then commands the robot to **pick** the cube and **place** it at a target location.

## Detection Node (Provided)

This node performs **cube pose estimation** and **publishes poses to ROS 2 topics**.

**Command:**

```bash
ros2 run rosconuk25_ope PositionEstimation.py environment:="<ENV>" model:=ColouredCubes_ur3 visualize:=true
# ENV  = gazebo | robot
```

After launching it, you can verify the topics and messages from the terminal:

```bash
ros2 topic list
ros2 topic echo /BlueCube/ObjectPoseEstimation
ros2 topic echo /GreenCube/ObjectPoseEstimation
ros2 topic echo /RedCube/ObjectPoseEstimation
ros2 topic echo /WhiteCube/ObjectPoseEstimation
```

> Each `/ColorCube/ObjectPoseEstimation` topic publishes the **estimated pose** for that cube.

## cubePP_detection.py

Now you will focus on the Python script that **uses the detected pose** to execute the pick & place.

**Objective:**  

Write a script that selects a cube (by name), **subscribes** to its pose topic, waits for a valid pose, and then performs: approach → pick → retract → move to place → place → retract. The pose values coming from the topic will be the **inputs** to your robot movements.

**How to run your script:**

```bash
ros2 run rosconuk25_ope cubePP_detection.py environment:="<ENV>" cube:="<CUBE>"
# ENV  = gazebo | robot
# CUBE = BlueCube | WhiteCube | RedCube | GreenCube
```

**What your script must do:**

- **Subscribe** to the correct `/ColorCube/ObjectPoseEstimation` topic and read the latest cube pose.
- **Use the received position** (and a fixed orientation from Exercise C) to:
  - Move above the cube (**approach**).
  - Move down to the cube (**pick**) and close the gripper.
  - Retract to approach.
  - Move to a predefined **place** approach pose.
  - Move down to **place**, open the gripper, and retract.
- Use the same robot operation methods/classes from Exercise C (MoveJ/MoveL + gripper control).

> Tip: Start in **Gazebo** (`environment:=gazebo`) to validate your logic and speeds before attempting `environment:=robot`.