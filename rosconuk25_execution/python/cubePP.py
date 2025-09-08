#!/usr/bin/python3
import sys
sys.dont_write_bytecode = True

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: April, 2025.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statements:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.
# IFRA-Cranfield (2025) ROS 2 Sim-to-Real Robot Control. UR3 Robot. URL: https://github.com/IFRA-Cranfield/ur3_CranfieldRobotics.
# IFRA-Cranfield (2025) Robot Simulation and Control Workshop, ROSCon UK 2025. URL: https://github.com/IFRA-Cranfield/ROSCon_UK_2025.

# CubePP.py
# This program performs a Cube Pick & Place task.

# ===== IMPORT REQUIRED COMPONENTS ===== #

# System functions and classes:
import sys, os, time

# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# IMPORT ROS2 Custom Messages:
from objectpose_msgs.msg import ObjectPose
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Xyzypr
from ros2srrc_data.msg import Ypr
from ros2srrc_data.msg import Robpose

# IMPORT Python classes:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_robot = PATH + "/robot"
PATH_endeffector = PATH + "/endeffector"
PATH_endeffector_gz = PATH + "/endeffector_gz"
# ROBOT CLASS:
sys.path.append(PATH_robot)
from robot import RBT
# END EFFECTOR CLASSES (Gazebo):
sys.path.append(PATH_endeffector)
from robotiq_ur import RobotiqGRIPPER
# END EFFECTOR CLASSES (Gazebo):
sys.path.append(PATH_endeffector_gz)
from parallelGripper import parallelGR

# ==================================================================== #  
# ==================================================================== # 

# ===== EVALUATE INPUT ARGUMENTS ===== #         
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# ===== CLOSE PROGRAM function ===== #
def close():

    rclpy.shutdown()
    print("")
    print("CLOSING PROGRAM... BYE!")
    exit()

# ==================================================================== #  
# ==================================================================== # 

def main(args=None):

    rclpy.init()

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")
    print(" ROSCon UK 2025 IFRA Workshop ")
    print("")

    print("Object Detection and Pose Estimation in ROS 2.")
    print("Python script -> cubePP.py")
    print("")

    # (1)
    # Firstly, we'll get the ENVIRONMENT variable to check whether we are in simulation or real world:
    ENVIRONMENT = AssignArgument("environment")
    if ENVIRONMENT == "gazebo" or ENVIRONMENT == "robot":
        print("Environment selected -> "+ ENVIRONMENT)
    else:
        print("")
        print("ERROR: environment INPUT ARGUMENT has not been properly defined (gazebo/robot). Please try again.")
        print("Closing... BYE!")
        exit()

    # (2)
    # We'll initialise the ROBOT and END-EFFECTOR Python Classes:

    # ROBOT:
    ROBOT = RBT()

    # END-EFFECTOR:
    if ENVIRONMENT == "gazebo":
        ENDEFFECTOR = parallelGR(["RedCube", "BlueCube", "WhiteCube", "GreenCube"], "ur3", "EE_robotiq_hande")
    else:
        ENDEFFECTOR = RobotiqGRIPPER()

    # (3)
    # We'll perform the PICK & PLACE TASK:

    # 1. MoveJ to HomePosition:
    print(" ===== [CUBE PP Task]: STEP N-1 ===== ")
    print("Moving the UR3 robot to -> HomePosition...")
    print("")

    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = 0.25
    
    INPUT = Joints()
    INPUT.joint1 = 90.0
    INPUT.joint2 = -90.0
    INPUT.joint3 = 90.0
    INPUT.joint4 = -90.0
    INPUT.joint5 = -90.0
    INPUT.joint6 = 0.0
    ACTION.movej = INPUT

    RES = ROBOT.Move_EXECUTE(ACTION)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # 2. PTP-RobMove to PickApproach:
    print(" ===== [CUBE PP Task]: STEP N-2 ===== ")
    print("Moving the UR3 robot to -> PickApproach")
    print("")

    MovType = "PTP"
    speed = 0.25
    InputPose = Robpose()
    InputPose.x = -0.2076
    InputPose.y = 0.1903
    InputPose.z = 0.925
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    RES = ROBOT.RobMove_EXECUTE(MovType, speed, InputPose)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # 3. LIN-RobMove to Pick:
    print(" ===== [CUBE PP Task]: STEP N-3 ===== ")
    print("Moving the UR3 robot to -> Pick")
    print("")

    MovType = "LIN"
    speed = 0.1
    InputPose = Robpose()
    InputPose.x = -0.2076
    InputPose.y = 0.1903
    InputPose.z = 0.8625
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    RES = ROBOT.RobMove_EXECUTE(MovType, speed, InputPose)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # Small delay before grasping:
    time.sleep(0.5)

    # 4. CLOSE GRIPPER:
    print(" ===== [CUBE PP Task]: STEP N-4 ===== ")
    print("Gripper -> CLOSE")
    print("")

    if ENVIRONMENT == "gazebo":
        RES = ENDEFFECTOR.CLOSE(25.0)
    else:
        RES = ENDEFFECTOR.CLOSE()

    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # Small delay after grasping:
    time.sleep(0.5)

    # 5. LIN-RobMove to PickApproach:
    print(" ===== [CUBE PP Task]: STEP N-5 ===== ")
    print("Moving the UR3 robot to -> PickApproach")
    print("")

    MovType = "LIN"
    speed = 0.1
    InputPose = Robpose()
    InputPose.x = -0.2076
    InputPose.y = 0.1903
    InputPose.z = 0.925
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    RES = ROBOT.RobMove_EXECUTE(MovType, speed, InputPose)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # 6. MoveJ to IntermediatePosition:
    print(" ===== [CUBE PP Task]: STEP N-6 ===== ")
    print("Moving the UR3 robot to -> IntermediatePosition...") # This is done to avoid PTP-RobMove planning errors between PickApproach-PlaceApproach.
    print("")

    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = 0.25
    
    INPUT = Joints()
    INPUT.joint1 = 90.0
    INPUT.joint2 = -90.0
    INPUT.joint3 = 90.0
    INPUT.joint4 = -90.0
    INPUT.joint5 = -90.0
    INPUT.joint6 = 0.0
    ACTION.movej = INPUT

    RES = ROBOT.Move_EXECUTE(ACTION)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("") 

    # 7. PTP-RobMove to PlaceApproach:
    print(" ===== [CUBE PP Task]: STEP N-7 ===== ")
    print("Moving the UR3 robot to -> PlaceApproach")
    print("")

    MovType = "PTP"
    speed = 0.25
    InputPose = Robpose()
    InputPose.x = 0.2082
    InputPose.y = 0.1764
    InputPose.z = 0.975
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    RES = ROBOT.RobMove_EXECUTE(MovType, speed, InputPose)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # 8. LIN-RobMove to Place:
    print(" ===== [CUBE PP Task]: STEP N-8 ===== ")
    print("Moving the UR3 robot to -> Place")
    print("")

    MovType = "LIN"
    speed = 0.1
    InputPose = Robpose()
    InputPose.x = 0.2082
    InputPose.y = 0.1764
    InputPose.z = 0.925
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    RES = ROBOT.RobMove_EXECUTE(MovType, speed, InputPose)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # Small delay before releasing:
    time.sleep(0.5)

    # 9. OPEN GRIPPER:
    print(" ===== [CUBE PP Task]: STEP N-9 ===== ")
    print("Gripper -> OPEN")
    print("")

    RES = ENDEFFECTOR.OPEN()

    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # Small delay after releasing:
    time.sleep(0.5)

    # 10. LIN-RobMove to PlaceApproach:
    print(" ===== [CUBE PP Task]: STEP N-10 ===== ")
    print("Moving the UR3 robot to -> PlaceApproach")
    print("")

    MovType = "LIN"
    speed = 0.1
    InputPose = Robpose()
    InputPose.x = 0.2082
    InputPose.y = 0.1764
    InputPose.z = 0.975
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    RES = ROBOT.RobMove_EXECUTE(MovType, speed, InputPose)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # 11. MoveJ to HomePosition:
    print(" ===== [CUBE PP Task]: STEP N-11 ===== ")
    print("Moving the UR3 robot to -> HomePosition...")
    print("")

    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = 0.25
    
    INPUT = Joints()
    INPUT.joint1 = 90.0
    INPUT.joint2 = -90.0
    INPUT.joint3 = 90.0
    INPUT.joint4 = -90.0
    INPUT.joint5 = -90.0
    INPUT.joint6 = 0.0
    ACTION.movej = INPUT

    RES = ROBOT.Move_EXECUTE(ACTION)
    if RES["Success"]:
        print("Action executed!")
        print("Result -> " + RES["Message"])
    else:
        print("ERROR -> " + RES["Message"])
        close()
    print("")

    # ============================ #

    print("CUBE PICK & PLACE task successfully executed.")
    print("")

    rclpy.shutdown()
    print("CLOSING PROGRAM... BYE!")
    exit()

if __name__ == '__main__':
    main()