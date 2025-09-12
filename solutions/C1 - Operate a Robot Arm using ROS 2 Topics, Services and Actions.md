# PART C: Integration of External Tools within the ROS 2-based Robotic System

## C1: Operate a Robot Manipulator using ROS 2 Topics, Services and Actions

__COMMANDS__

Robot Movements:
```sh
# MoveJ:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}"
# MoveL:
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.00}, speed: 1.0}"
# MoveR:
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

Robot State Monitoring:
```sh
# To check the state of the joints:
ros 2 topic echo /joint_states

# To check the end-effector pose:
ros2 topic echo /Robpose
```

__SOLUTION__

1. Launch the ROSConUK25 UR3 Robot Cell's Gazebo + MoveIt!2 Simulation Environment:
    ```sh
    ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_2  # UR3 + Robotiq HandE Gripper.
    ```

2. In 2 different terminal shells, execute the Robot Monitoring ROS 2 Topic Subscribers.
    ```sh
    ros 2 topic echo /joint_states
    ros2 topic echo /Robpose
    ```

3. In a new terminal shell, execute the following movements:
    ```sh
    # MoveJ:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 90.00, joint2: -90.00, joint3: 90.00, joint4: -90.00, joint5: -90.00, joint6: 0.00}, speed: 1.0}"

    # MoveL:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: -0.15}, speed: 0.1}"

    # MoveL:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.15}, speed: 1.0}"

    # MoveR:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveR', mover: {joint: 'joint1', value: 45.00}, speed: 1.0}"

    # MoveR:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveR', mover: {joint: 'joint1', value: -45.00}, speed: 0.25}"

    # MoveROT:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.00, pitch: 90.00, roll: 0.00}, speed: 1.0}"

    # MoveROT:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.00, pitch: -90.00, roll: 0.00}, speed: 1.0}"

    # MoveRP:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.0, y: 0.00, z: 0.05, yaw: 0.00, pitch: 0.00, roll: 30.0}, speed: 1.0}"

    # MoveRP:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.0, y: 0.00, z: 0.05, yaw: 0.00, pitch: 0.00, roll: -30.0}, speed: 1.0}"
    ```