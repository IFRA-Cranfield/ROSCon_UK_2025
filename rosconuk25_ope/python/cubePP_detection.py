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

# cubePP_detection.py
# This program:
#   1. Checks if the desired CUBE is within the workspace.
#   2. Sends its coordinates to the robot to perform a pick and place task.

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

# ===== CUBE POSE - SUBSCRIBER NODE ===== #  
class CubePose(Node):

    def __init__(self, OBJECT):

        super().__init__("rosconuk25_ope_cubepose")
        
        TopicName = "/" + OBJECT + "/ObjectPoseEstimation"
        self.SUB = self.create_subscription(ObjectPose, TopicName, self.CALLBACK_FN, 10)

        self.POSE = ObjectPose()
        self.FOUND = False

    def CALLBACK_FN(self, POSE):
        
        self.POSE = POSE
        self.FOUND = True

    def GetCubePose(self):

        T = time.time() + 3.0
        while time.time() < T:
            
            rclpy.spin_once(self, timeout_sec=0.5)

            if self.FOUND == True:
                
                self.FOUND = False
                return({"Success": True, "Pose": self.POSE})
            
        return({"Success": False, "Pose": None})

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
    print("Python script -> cubePP_detection.py")
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
    # Secondly, we'll get the CUBE variable to select the CUBE that is going to be picked by the robot:
    CUBEname = AssignArgument("cube")
    if CUBEname == "BlueCube" or CUBEname == "GreenCube" or CUBEname == "RedCube" or CUBEname == "WhiteCube":
        print("CUBE selected -> "+ CUBEname)
    else:
        print("")
        print("ERROR: cube INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()

    # (3)
    # We'll initialise the ROBOT and END-EFFECTOR Python Classes:

    # ROBOT:
    ROBOT = RBT()

    # END-EFFECTOR:
    if ENVIRONMENT == "gazebo":
        ENDEFFECTOR = parallelGR(["RedCube", "BlueCube", "WhiteCube", "GreenCube"], "ur3", "EE_robotiq_hande")
    else:
        ENDEFFECTOR = RobotiqGRIPPER()

    # (4)
    # We'll initialise the ROS 2 Node (Subscriber) that will give us the ObjectPose:
    OBJPOSE = CubePose(CUBEname)

    # (5)
    # We'll perform the PICK & PLACE TASK:

    # 1. Move ROBOT to the Initial Position:
    print(" ===== [CUBE PP - Detection Task]: STEP N-1 ===== ")
    print("Moving the UR3 robot to Initial Position...") # This Pose gives us clearance to see the whole ARUCO GRID.
    print("")

    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = 0.25
    
    INPUT = Joints()
    INPUT.joint1 = 180.0
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

    # 2. Check if the cube is within the workspace:
    print(" ===== [CUBE PP - Detection Task]: STEP N-2 ===== ")
    print("Checking if the selected CUBE is detected within the robot workspace...")
    print("")

    # ADD HERE.

    # =========== Pick and place task =========== #

    # 3. Move ROBOT to Intermediate Position:
    print(" ===== [CUBE PP - Detection Task]: STEP N-3 ===== ")
    print("Moving the UR3 robot to Intermediate Position...")
    print("")

    # ADD HERE.

    # 4. PickApproach:
    print(" ===== [CUBE PP - Detection Task]: STEP N-4 ===== ")
    print("Moving the UR3 robot to -> PickApproach")
    print("")

    # ADD HERE.

    # 5. Pick:
    print(" ===== [CUBE PP - Detection Task]: STEP N-5 ===== ")
    print("Moving the UR3 robot to -> Pick")
    print("")

    # ADD HERE.

    # Small delay before grasping:
    time.sleep(0.5)

    # 6. CLOSE GRIPPER:
    print(" ===== [CUBE PP - Detection Task]: STEP N-6 ===== ")
    print("Gripper -> CLOSE")
    print("")

    # ADD HERE.

    # Small delay after grasping:
    time.sleep(0.5)

    # 7. PickApproach:
    print(" ===== [CUBE PP - Detection Task]: STEP N-7 ===== ")
    print("Moving the UR3 robot to -> PickApproach")
    print("")

    # ADD HERE.

    # 8. PlaceApproach:
    print(" ===== [CUBE PP - Detection Task]: STEP N-8 ===== ")
    print("Moving the UR3 robot to -> PlaceApproach")
    print("")

    # ADD HERE.

    # 9. Place:
    print(" ===== [CUBE PP - Detection Task]: STEP N-9 ===== ")
    print("Moving the UR3 robot to -> Place")
    print("")

    # ADD HERE.

    # Small delay before releasing:
    time.sleep(0.5)

    # 10. OPEN GRIPPER:
    print(" ===== [CUBE PP - Detection Task]: STEP N-10 ===== ")
    print("Gripper -> OPEN")
    print("")

    # ADD HERE.

    # Small delay after releasing:
    time.sleep(0.5)

    # 11. PlaceApproach:
    print(" ===== [CUBE PP - Detection Task]: STEP N-11 ===== ")
    print("Moving the UR3 robot to -> PlaceApproach")
    print("")

    # ADD HERE.

    # 12. Move ROBOT to Initial Position:
    print(" ===== [CUBE PP - Detection Task]: STEP N-12 ===== ")
    print("Moving the UR3 robot to Initial Position...")
    print("")

    # ADD HERE.

    # ============================ #

    print("CUBE pick & place task successfully executed.")
    print("")

    rclpy.shutdown()
    print("CLOSING PROGRAM... BYE!")
    exit()

if __name__ == '__main__':
    main()