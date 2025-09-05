TBD

ros2 launch ros2srrc_launch moveit2.launch.py package:=rosconuk25 config:=ur3_3

ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 180.00, joint2: -90.00, joint3: 90.00, joint4: -90.00, joint5: -90.00, joint6: 0.00}, speed: 1.0}"

ros2 run ur3cranfield_ope PositionEstimation.py environment:=gazebo model:=ColouredCubes_ur3 visualize:=true
pip install "numpy<2.0"

ros2 run rosconuk25_ope cubePP_detection.py environment:=gazebo cube:=BlueCube