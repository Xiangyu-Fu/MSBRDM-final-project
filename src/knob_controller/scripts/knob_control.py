#!/usr/bin/env python3
import rospy
from impedance_controller.srv import MoveArmCartesian, MoveArmJoint
import math
import numpy as np
from geometry_msgs.msg import WrenchStamped

def FT_callback(data):
    global force_torque_matrix
    force_torque_matrix[0] = data.wrench.force.x
    force_torque_matrix[1] = data.wrench.force.y
    force_torque_matrix[2] = data.wrench.force.z
    force_torque_matrix[3] = data.wrench.torque.x
    force_torque_matrix[4] = data.wrench.torque.y
    force_torque_matrix[5] = data.wrench.torque.z

def FT_listener():
    rospy.init_node('knob_control', anonymous=True)
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

    FT_listener()
    force_torque_matrix = np.zeros((6, 1))

    # # Example call to moveArmCartesian service
    # response_cartesian = move_arm_cartesian_client(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
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
        response_cartesian = move_arm_cartesian_client(traget_pos[0], traget_pos[1], traget_pos[2], traget_pos[3], traget_pos[4], traget_pos[5])
        # print("MoveArmCartesian response:", response_cartesian)

        rospy.loginfo("Updated force_torque_matrix in main:\n%s", force_torque_matrix)
        
        rospy.sleep(0.05)


    # print("MoveArmCartesian response:", response_cartesian)
    
    # Example call to moveArmJoint service with sinusoidal wave added
    t = 0
    frequency = 0.5  # Adjust frequency as needed
    while not rospy.is_shutdown():
        t += 1  # Adjust time step as needed
        sinusoidal_offset = math.sin(frequency * t)
        joint0 = 0.0 + sinusoidal_offset
        joint1 = -1.57 + sinusoidal_offset
        joint2 = -1.57 + sinusoidal_offset
        joint3 = -1.57 + sinusoidal_offset
        joint4 = 1.57 + sinusoidal_offset
        joint5 = 0.0 + sinusoidal_offset
        response_joint = move_arm_joint_client(joint0, joint1, joint2, joint3, joint4, joint5)
        print("MoveArmJoint response:", response_joint)
        rospy.sleep(1)  # Adjust sleep time as needed

