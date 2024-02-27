#!/usr/bin/env python3
import sys
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState, KnobCommand
from threading import Lock
from ur10_ctrl_client import UR10CtrlClient
from geometry_msgs.msg import WrenchStamped, Pose, PoseStamped, Point, Quaternion, Vector3
from qt5_gui import Ui_MainWindow
from PyQt5 import QtCore, QtGui, QtWidgets
def callback(data):
    print(data.position)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()