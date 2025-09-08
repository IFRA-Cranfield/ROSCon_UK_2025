# Exercise C (PART 1): Robot Control via Actions, Topics, and Python

In this exercise you will control the UR3 robot using **ROS 2 Topics, Services & Actions**, and finally a **Python script** built on the `ros2_SimRealRobotControl` (ros2srrc) Python clients.

## Exercise C1 — Operate the Robot with ROS 2 Actions and Monitor the Robot Pose using a ROS 2 Topic

**Goal:**  

Use the `/Move` and `/RobMove` **ROS 2 Actions** to command motions, and monitor the robot’s real-time pose with the **`RobPose`** topic.

**Reference (follow these docs for exact commands & usage):**  
- Robot operation guide: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/RobotOperation.md

**What to do:**

1. Open the operation guide above.
2. Use the documented CLI commands to:
   - Send **Joint-Space** and **Cartesian-Space** moves via **`/Move`** and **`/RobMove`**.
   - Observe behavior differences (planning, speed parameters, etc.).
3. In another terminal, **monitor robot pose** (position + orientation) using the **`RobPose`** topic command shown in the same guide.

**You should see:**

- Action goals accepted and completed.
- Continuous pose updates on `RobPose` while the robot moves.

## Exercise C2 — Execute a Pick & Place Robot Program (YAML)

**Goal:**  

Create a **YAML robot program** (pick & place for UR3) and execute it with `ExecuteProgram.py`.

**References:**

- Program execution guide: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/ProgramExecution.md  
- Programs are located at: `/rosconuk25/execution/programs`  
- A template YAML with all supported motions is provided in the same folder.

**About `ExecuteProgram.py`:**  

It reads a YAML program from a ROS 2 package’s `/programs` folder and executes steps **sequentially** (e.g., `MoveJ`, `MoveR`, `MoveL`, gripper ops, delays…). The **Specifications** section selects the correct robot and tool clients.

**Command format:**

```sh
ros2 run ros2srrc_execution ExecuteProgram.py package:="PACKAGE_NAME" program:="PROGRAM_NAME"
# PACKAGE_NAME: the ROS 2 package that contains `/programs/PROGRAM_NAME.yaml`
# PROGRAM_NAME: YAML file name **without** `.yaml`
```

### Required poses & orientation (ESSENTIAL)

**Pick pose (UR3):**

- `x = -0.2076`, `y = 0.1903`
- `z = 0.925` (**approach**), `z = 0.8625` (**pick**)

**Place pose (UR3):**

- `x = 0.2082`, `y = 0.1764`
- `z = 0.975` (**approach**), `z = 0.925` (**place**)

**Orientation (quaternion) — use **everywhere**:**

- `qx = 0.0`, `qy = 1.0`, `qz = 0.0`, `qw = 0.0`

### Steps

1. Copy the **template YAML** from `/rosconuk25/execution/programs` and rename it, e.g.:
   - `cubePP_ur3.yaml`
2. Edit the YAML **Specifications** tags.
3. Add steps to:
   - Move above the **pick** pose (approach `z=0.925`), same `x,y`.
   - Descend to **pick** pose (`z=0.8625`), close gripper.
   - Retract to **pick approach** height.
   - Move above the **place** pose (approach `z=0.975`), same `x,y`.
   - Descend to **place** pose (`z=0.925`), open gripper.
   - Retract to **place approach** height.
   - Remember to add the open/close gripper commands! (NOTE: The commands are different in simulation/real robot)

4. Execute your program:
   ```
   ros2 run ros2srrc_execution ExecuteProgram.py package:=rosconuk25_execution program:="---"
   ```

**Validation:**

- Robot picks from the **pick** coordinates/orientation.
- Robot places at the **place** coordinates/orientation.
- Gripper closes/opens at the correct moments.

## Exercise C3 — Implement the Same Pick & Place with a Python Script

**Goal:**  

Write a **Python script** that performs the same pick & place as C2, using ros2srrc Python clients.

**Where to put your script:**  

- Path: `/rosconuk25_execution/python/cubePP.py`

**How to run:**

```sh
ros2 run rosconuk25_execution cubePP.py environment:="---"
# "environment" must be gazebo or robot.
```

**References (clients & examples):**

- Python clients & usage examples: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/ros2srrc_execution/python/testClients

### Steps

1. Review the client examples in `testClients` to see how to:
   - Initialize the action/topic clients.
   - Send **MoveJ/MoveL/RobMove** (or equivalent) commands.
   - Control the **gripper** (open/close).
   - Optionally read **RobPose** for feedback/logging.

2. Implement `cubePP.py` to:
   - Initialize the execution context for `ENV` (robot/gazebo).
   - Move to **pick approach** → **pick** → close gripper → retract.
   - Move to **place approach** → **place** → open gripper → retract.
   - Use the **same poses & orientation** as in C2.

**Required poses & orientation (repeat):**

- **Pick:**  
  `(-0.2076, 0.1903, 0.925)` approach → `(-0.2076, 0.1903, 0.8625)` pick  
- **Place:**  
  `(0.2082, 0.1764, 0.975)` approach → `(0.2082, 0.1764, 0.925)` place  
- **Orientation (all moves):**  
  `qx=0.0, qy=1.0, qz=0.0, qw=0.0`

**Validation:**

- Script performs the same sequence as your YAML program.
- Works for both `ENV=gazebo` and `ENV=robot` (be cautious on real hardware).

**Safety note:**  

On **real hardware**, test speeds conservatively and keep a hand on the e-stop.