#!/usr/bin/env python3
import rospy
from impedance_controller.srv import MoveArmCartesian, MoveArmJoint
import math

class UR10CtrlClient:
    def __init__(self):
        self.home_pos = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0] 
        self.home_joint = [ math.radians(0),
                            math.radians(-70),
                            math.radians(-110),
                            math.radians(-90),
                            math.radians(90),
                            math.radians(0)]

    def move_arm_cartesian(self, x, y, z, rx, ry, rz) -> str:
        rospy.wait_for_service('move_arm_cartesian')
        try:
            move_arm_cartesian = rospy.ServiceProxy('move_arm_cartesian', MoveArmCartesian)
            response = move_arm_cartesian(x, y, z, rx, ry, rz)
            return response
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def move_arm_joint(self, joint0, joint1, joint2, joint3, joint4, joint5) -> str:
        rospy.wait_for_service('move_arm_joint')
        try:
            move_arm_joint = rospy.ServiceProxy('move_arm_joint', MoveArmJoint)
            response = move_arm_joint(joint0, joint1, joint2, joint3, joint4, joint5)
            return response
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def move_arm_cartesian_home(self) -> None:
        self.move_arm_cartesian(self.home_pos[0],
                                self.home_pos[1],
                                self.home_pos[2],
                                self.home_pos[3],
                                self.home_pos[4],
                                self.home_pos[5])

    def move_arm_cartesian_joint(self) -> None: 
        self.move_arm_joint(self.home_joint[0],
                                    self.home_joint[1],
                                    self.home_joint[2],
                                    self.home_joint[3],
                                    self.home_joint[4],
                                    self.home_joint[5])
        
    def move_arm_cartesian_test(self) -> None:
        traget_pos = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0]
        frequency = 0.05  # Adjust frequency as needed]
        t = 0
        while not rospy.is_shutdown():
            t += 1
            sinusoidal_offset = 0.1*math.sin(frequency * t)
            traget_pos[0] = 0.5 + sinusoidal_offset
            traget_pos[1] = 0.0 + sinusoidal_offset
            traget_pos[2] = 0.5 + sinusoidal_offset
            print("traget_pos:", traget_pos)
            response_cartesian = self.move_arm_cartesian(traget_pos[0], traget_pos[1], traget_pos[2], traget_pos[3], traget_pos[4], traget_pos[5])
            # print("MoveArmCartesian response:", response_cartesian)
            rospy.sleep(0.05)