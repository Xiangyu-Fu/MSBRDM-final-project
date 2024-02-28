#!/usr/bin/env python3
import rospy
from impedance_controller.srv import MoveArmCartesian, MoveArmJoint
import math
import numpy as np
from geometry_msgs.msg import WrenchStamped

# global variable
global force_torque_matrix
force_torque_matrix = np.zeros((6, 1))

def FT_callback(data):
    force_torque_matrix[0] = data.wrench.force.x
    force_torque_matrix[1] = data.wrench.force.y
    force_torque_matrix[2] = data.wrench.force.z
    force_torque_matrix[3] = data.wrench.torque.x
    force_torque_matrix[4] = data.wrench.torque.y
    force_torque_matrix[5] = data.wrench.torque.z

def FT_listener():
    rospy.Subscriber("/schunk_netbox/raw", WrenchStamped, FT_callback)


def move_arm_cartesian_client(x, y, z, rx, ry, rz):
    rospy.wait_for_service('move_arm_cartesian')
    try:
        move_arm_cartesian = rospy.ServiceProxy('move_arm_cartesian', MoveArmCartesian)
        response = move_arm_cartesian(x, y, z, rx, ry, rz)
        return response
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def move_arm_joint_client(joint0, joint1, joint2, joint3, joint4, joint5):
    rospy.wait_for_service('move_arm_joint')
    try:
        move_arm_joint = rospy.ServiceProxy('move_arm_joint', MoveArmJoint)
        response = move_arm_joint(joint0, joint1, joint2, joint3, joint4, joint5)
        return response
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node('arm_control_client')

    # FT_listener()

    # Example call to moveArmCartesian service
    traget_pos = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0]
    frequency = 0.05  # Adjust frequency as needed]
    t = 0
    fac = 1
    while not rospy.is_shutdown():
        sinusoidal_offset = 0.05*math.sin(frequency * t)
        traget_pos[0] = 0.47
        traget_pos[1] = -0.16 
        traget_pos[2] = 0.6  + 0.01 * t
        print("traget_pos:", traget_pos)
        response_cartesian = move_arm_cartesian_client(traget_pos[0], traget_pos[1], traget_pos[2], traget_pos[3], traget_pos[4], traget_pos[5])
        # print("MoveArmCartesian response:", response_cartesian)
        if t == 20:
            fac = -1
        elif t == 0:
            fac = 1
        t= t + fac
        rospy.sleep(0.2)

    # Example call to moveArmJoint service with sinusoidal wave added
    t = 0
    frequency = 0.5  # Adjust frequency as needed
    while not rospy.is_shutdown():
        t += 1  # Adjust time step as needed
        sinusoidal_offset = 0.1*math.sin(frequency * t)
        #   init_q: [0, -70, -110, -90, 90, 0]
        joint0 = 0.0
        joint1 = math.radians(-70.0)
        joint2 = math.radians(-110.0) 
        joint3 = math.radians(-90.0)
        joint4 = math.radians(90.0) 
        joint5 = 0.0
        response_joint = move_arm_joint_client(joint0, joint1, joint2, joint3, joint4, joint5)
        print("MoveArmJoint response:", response_joint)
        rospy.sleep(0.1)  # Adjust sleep time as needed

