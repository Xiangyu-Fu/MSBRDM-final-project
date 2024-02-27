#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2024-02-26

import sys
import math
import rospy
from knob_robot_control.msg import KnobState, KnobCommand
from std_msgs.msg import String, Int32, Float32
from threading import Lock, Thread
from geometry_msgs.msg import WrenchStamped, Pose, PoseStamped, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from impedance_controller.srv import MoveArmCartesian, MoveArmJoint

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QDoubleSpinBox, QLabel, QLineEdit
# from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from PyQt5.QtGui import QPainter
from PyQt5.QtCore import Qt


class Ui_MainWindow(object):
    def __init__(self):
        self.knob_current_pos = None
        self.knob_current_force = None
        self.tcp_wrench = None

        self.ur10_joint_cur = None
        self.ur10_cart_cur = None

        self.ur10_cart_start = None
        self.ur10_joint_start = None

        self.home_pos = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0] 
        self.home_joint = [ math.radians(0),
                            math.radians(-70),
                            math.radians(-110),
                            math.radians(-90),
                            math.radians(90),
                            math.radians(0)]
        
    """
    ROS SETUP
    """
    def setup_ROS(self):
        # define the subscibers
        rospy.init_node("knob_gui")

        self.knob_state_sub = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback, queue_size=1)
        # self.tcp_wrench_sub = rospy.Subscriber("/tcp_wrench", WrenchStamped, self.tcp_wrench_callback)

        # ur10 sub
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=1)
        self.cart_state_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self.tcp_state_callback, queue_size=1)

        self.knob_command_pub = rospy.Publisher("/knob_command", KnobCommand, queue_size=1)

        # # Create a new thread to run the ROS spin loop
        # ros_thread = Thread(target=rospy.spin)
        # ros_thread.start()

        # shutdown the ros node when the window is closed
        # rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.signal_shutdown("shutdown")

    """
    UI SETUP
    """
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(240, 20, 20, 521))
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(50, 520, 131, 25))
        self.pushButton_2.setObjectName("pushButton_2")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(10, 20, 151, 17))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(260, 20, 151, 17))
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setGeometry(QtCore.QRect(260, 180, 151, 17))
        self.label_11.setObjectName("label_11")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(390, 460, 131, 25))
        self.pushButton_3.setObjectName("pushButton_3")
        self.line_2 = QtWidgets.QFrame(self.centralwidget)
        self.line_2.setGeometry(QtCore.QRect(650, 20, 20, 521))
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(310, 50, 315, 102))
        self.widget.setObjectName("widget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_7 = QtWidgets.QLabel(self.widget)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout.addWidget(self.label_7)
        self.text_line_edit = QtWidgets.QDoubleSpinBox(self.widget)
        self.text_line_edit.setEnabled(True)
        self.text_line_edit.setReadOnly(True)
        self.text_line_edit.setObjectName("text_line_edit")
        self.text_line_edit.setMinimum(-10)
        self.horizontalLayout.addWidget(self.text_line_edit)
        self.doubleSpinBox_8 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_8.setEnabled(True)
        self.doubleSpinBox_8.setReadOnly(True)
        self.doubleSpinBox_8.setObjectName("doubleSpinBox_8")
        self.doubleSpinBox_8.setMinimum(-10)
        self.horizontalLayout.addWidget(self.doubleSpinBox_8)
        self.doubleSpinBox_9 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_9.setEnabled(True)
        self.doubleSpinBox_9.setReadOnly(True)
        self.doubleSpinBox_9.setObjectName("doubleSpinBox_9")
        self.doubleSpinBox_9.setMinimum(-10)
        self.horizontalLayout.addWidget(self.doubleSpinBox_9)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_8 = QtWidgets.QLabel(self.widget)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_4.addWidget(self.label_8)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.doubleSpinBox_14 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_14.setEnabled(True)
        self.doubleSpinBox_14.setReadOnly(True)
        self.doubleSpinBox_14.setObjectName("doubleSpinBox_14")
        self.doubleSpinBox_14.setMinimum(-10) 
        self.horizontalLayout_2.addWidget(self.doubleSpinBox_14)
        self.doubleSpinBox_15 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_15.setEnabled(True)
        self.doubleSpinBox_15.setReadOnly(True)
        self.doubleSpinBox_15.setObjectName("doubleSpinBox_15")
        self.doubleSpinBox_15.setMinimum(-10) 
        self.horizontalLayout_2.addWidget(self.doubleSpinBox_15)
        self.doubleSpinBox_13 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_13.setEnabled(True)
        self.doubleSpinBox_13.setReadOnly(True)
        self.doubleSpinBox_13.setObjectName("doubleSpinBox_13")
        self.doubleSpinBox_13.setMinimum(-10)
        self.horizontalLayout_2.addWidget(self.doubleSpinBox_13)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.doubleSpinBox_16 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_16.setEnabled(True)
        self.doubleSpinBox_16.setReadOnly(True)
        self.doubleSpinBox_16.setObjectName("doubleSpinBox_16")
        self.doubleSpinBox_16.setMinimum(-10)
        self.horizontalLayout_3.addWidget(self.doubleSpinBox_16)
        self.doubleSpinBox_17 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_17.setEnabled(True)
        self.doubleSpinBox_17.setReadOnly(True)
        self.doubleSpinBox_17.setObjectName("doubleSpinBox_17")
        self.doubleSpinBox_17.setMinimum(-10)
        self.horizontalLayout_3.addWidget(self.doubleSpinBox_17)
        self.doubleSpinBox_18 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_18.setEnabled(True)
        self.doubleSpinBox_18.setReadOnly(True)
        self.doubleSpinBox_18.setObjectName("doubleSpinBox_18")
        self.doubleSpinBox_18.setMinimum(-10)
        self.horizontalLayout_3.addWidget(self.doubleSpinBox_18)
        self.verticalLayout_2.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4.addLayout(self.verticalLayout_2)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.widget1 = QtWidgets.QWidget(self.centralwidget)
        self.widget1.setGeometry(QtCore.QRect(20, 50, 181, 411))
        self.widget1.setObjectName("widget1")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget1)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.widget1)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)

        self.num_positions_spinbox = QtWidgets.QDoubleSpinBox(self.widget1)
        self.num_positions_spinbox.setObjectName("num_positions_spinbox")

        self.verticalLayout.addWidget(self.num_positions_spinbox)
        self.label_2 = QtWidgets.QLabel(self.widget1)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.position_spinbox = QtWidgets.QDoubleSpinBox(self.widget1)
        self.position_spinbox.setObjectName("position_spinbox")
        self.verticalLayout.addWidget(self.position_spinbox)
        self.label_3 = QtWidgets.QLabel(self.widget1)
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        self.position_width_radians_spinbox = QtWidgets.QDoubleSpinBox(self.widget1)
        self.position_width_radians_spinbox.setObjectName("position_width_radians_spinbox")
        self.verticalLayout.addWidget(self.position_width_radians_spinbox)
        self.label_4 = QtWidgets.QLabel(self.widget1)
        self.label_4.setObjectName("label_4")
        self.verticalLayout.addWidget(self.label_4)
        self.detent_strength_unit_spinbox = QtWidgets.QDoubleSpinBox(self.widget1)
        self.detent_strength_unit_spinbox.setObjectName("detent_strength_unit_spinbox")
        self.verticalLayout.addWidget(self.detent_strength_unit_spinbox)
        self.label_5 = QtWidgets.QLabel(self.widget1)
        self.label_5.setObjectName("label_5")
        self.verticalLayout.addWidget(self.label_5)
        self.endstop_strength_unit_spinbox = QtWidgets.QDoubleSpinBox(self.widget1)
        self.endstop_strength_unit_spinbox.setObjectName("endstop_strength_unit_spinbox")
        self.verticalLayout.addWidget(self.endstop_strength_unit_spinbox)
        self.label_6 = QtWidgets.QLabel(self.widget1)
        self.label_6.setObjectName("label_6")
        self.verticalLayout.addWidget(self.label_6)
        self.snap_point_spinbox = QtWidgets.QDoubleSpinBox(self.widget1)
        self.snap_point_spinbox.setObjectName("snap_point_spinbox")
        self.verticalLayout.addWidget(self.snap_point_spinbox)
        self.pushButton = QtWidgets.QPushButton(self.widget1)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.widget2 = QtWidgets.QWidget(self.centralwidget)
        self.widget2.setGeometry(QtCore.QRect(310, 210, 114, 54))
        self.widget2.setObjectName("widget2")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget2)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.radioButton = QtWidgets.QRadioButton(self.widget2)
        self.radioButton.setObjectName("radioButton")
        self.verticalLayout_4.addWidget(self.radioButton)
        self.radioButton_2 = QtWidgets.QRadioButton(self.widget2)
        self.radioButton_2.setObjectName("radioButton_2")
        self.verticalLayout_4.addWidget(self.radioButton_2)
        self.widget3 = QtWidgets.QWidget(self.centralwidget)
        self.widget3.setGeometry(QtCore.QRect(310, 300, 114, 83))
        self.widget3.setObjectName("widget3")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget3)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.radioButton_5 = QtWidgets.QRadioButton(self.widget3)
        self.radioButton_5.setObjectName("radioButton_5")
        self.verticalLayout_5.addWidget(self.radioButton_5)
        self.radioButton_4 = QtWidgets.QRadioButton(self.widget3)
        self.radioButton_4.setObjectName("radioButton_4")
        self.verticalLayout_5.addWidget(self.radioButton_4)
        self.radioButton_3 = QtWidgets.QRadioButton(self.widget3)
        self.radioButton_3.setObjectName("radioButton_3")
        self.verticalLayout_5.addWidget(self.radioButton_3)
        self.widget4 = QtWidgets.QWidget(self.centralwidget)
        self.widget4.setGeometry(QtCore.QRect(490, 210, 74, 170))
        self.widget4.setObjectName("widget4")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.widget4)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.radioButton_7 = QtWidgets.QRadioButton(self.widget4)
        self.radioButton_7.setObjectName("radioButton_7")
        self.verticalLayout_6.addWidget(self.radioButton_7)
        self.radioButton_8 = QtWidgets.QRadioButton(self.widget4)
        self.radioButton_8.setObjectName("radioButton_8")
        self.verticalLayout_6.addWidget(self.radioButton_8)
        self.radioButton_6 = QtWidgets.QRadioButton(self.widget4)
        self.radioButton_6.setObjectName("radioButton_6")
        self.verticalLayout_6.addWidget(self.radioButton_6)
        self.radioButton_9 = QtWidgets.QRadioButton(self.widget4)
        self.radioButton_9.setObjectName("radioButton_9")
        self.verticalLayout_6.addWidget(self.radioButton_9)
        self.radioButton_10 = QtWidgets.QRadioButton(self.widget4)
        self.radioButton_10.setObjectName("radioButton_10")
        self.verticalLayout_6.addWidget(self.radioButton_10)
        self.radioButton_11 = QtWidgets.QRadioButton(self.widget4)
        self.radioButton_11.setObjectName("radioButton_11")
        self.verticalLayout_6.addWidget(self.radioButton_11)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1235, 22))
        self.menubar.setObjectName("menubar")
        self.menuHome = QtWidgets.QMenu(self.menubar)
        self.menuHome.setObjectName("menuHome")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuHome.menuAction())

        # add a chart
        self.widget5 = QtWidgets.QWidget(self.centralwidget)
        self.widget5.setGeometry(QtCore.QRect(700, 20, 500, 500))
        self.widget5.setObjectName("widget5")
        self.verticalLayout_chart = QtWidgets.QVBoxLayout(self.widget5)

        # self.chart = QChart()
        # self.chart.setTitle("Example Chart")
        # self.chart_view = QChartView(self.chart)
        # self.chart_view.setRenderHint(QPainter.Antialiasing)
        # self.verticalLayout_chart.addWidget(self.chart_view)

        # self.series1 = QLineSeries()
        # self.chart.addSeries(self.series1)
        # self.series1.setName("TCP Force")

        # self.series2 = QLineSeries()
        # self.chart.addSeries(self.series2)
        # self.series2.setName("Knob Force")

        # self.axisX = QValueAxis()
        # self.axisY = QValueAxis()
        # self.axisX.setTitleText("X Axis")
        # self.axisY.setTitleText("Y Axis")
        # self.chart.setAxisX(self.axisX, self.series1)
        # self.chart.setAxisY(self.axisY, self.series1)
        # self.chart.setAxisX(self.axisX, self.series2)
        # self.chart.setAxisY(self.axisY, self.series2)

        # self.axisX.setRange(0, 100) 
        # self.axisY.setRange(-10, 10)  
        
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow) -> None:
        """
        This function is responsible for setting up the user interface of the MainWindow.
        It sets the window title, ranges and default values for spinboxes, button callbacks, labels, and mode selection.
        """
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "UR10 Knob Control"))
        self.menuHome.setTitle(_translate("MainWindow", "Home"))

        # Set ranges and default values for spinboxes
        self.num_positions_spinbox.setRange(0, 100)
        self.num_positions_spinbox.setValue(11)
        self.position_spinbox.setRange(0, 100)
        self.position_spinbox.setValue(0)
        self.position_width_radians_spinbox.setRange(0, 100)
        self.position_width_radians_spinbox.setValue(10 * math.pi / 180)
        self.detent_strength_unit_spinbox.setRange(0, 100)
        self.detent_strength_unit_spinbox.setValue(0)
        self.endstop_strength_unit_spinbox.setRange(0, 100)
        self.endstop_strength_unit_spinbox.setValue(1)
        self.snap_point_spinbox.setRange(0, 100)
        self.snap_point_spinbox.setValue(1.1)

        # Button callbacks
        self.pushButton.setText(_translate("MainWindow", "Publish"))
        self.pushButton.clicked.connect(self.publish_knob_command)
        self.pushButton_2.setText(_translate("MainWindow", "Force Testing"))
        self.pushButton_2.clicked.connect(self.force_testing)
        self.pushButton_3.setText(_translate("MainWindow", "Mode Publish"))
        self.pushButton_3.clicked.connect(self.mode_change_callback)

        # Labels
        self.label_9.setText(_translate("MainWindow", "Mode Publisher"))
        self.label_10.setText(_translate("MainWindow", "State Monitor"))
        self.label_11.setText(_translate("MainWindow", "Control Mode"))
        self.label_7.setText(_translate("MainWindow", "Cartesian"))
        self.label_8.setText(_translate("MainWindow", "Joints"))
        self.label.setText(_translate("MainWindow", "Num Positions"))
        self.label_2.setText(_translate("MainWindow", "Positions"))
        self.label_3.setText(_translate("MainWindow", "Position Width Radians"))
        self.label_4.setText(_translate("MainWindow", "Detent Strength Unit"))
        self.label_5.setText(_translate("MainWindow", "Endstop Strength Unit"))
        self.label_6.setText(_translate("MainWindow", "Text"))

        # Mode selection
        self.radioButton.setText(_translate("MainWindow", " JOINT"))
        self.radioButton_2.setText(_translate("MainWindow", " CART"))
        self.radioButton_2.setChecked(True)

        # EE selection
        self.radioButton_5.setText(_translate("MainWindow", "X"))
        self.radioButton_4.setText(_translate("MainWindow", "Y"))
        self.radioButton_3.setText(_translate("MainWindow", "Z"))
        self.radioButton_5.setChecked(True)

        # Joint selection
        self.radioButton_7.setText(_translate("MainWindow", "Joint 1"))
        self.radioButton_8.setText(_translate("MainWindow", "Joint 2"))
        self.radioButton_6.setText(_translate("MainWindow", "Joint 3"))
        self.radioButton_9.setText(_translate("MainWindow", "Joint 4"))
        self.radioButton_10.setText(_translate("MainWindow", "Joint 5"))
        self.radioButton_11.setText(_translate("MainWindow", "Joint 6"))
        self.radioButton_7.setChecked(True)

    """
    UPDATE CHART(DISCARDED)
    """
    def update_chart_tcp(self, value) -> None:
        """
        Update the chart with a new EE value.

        Args:
            value (float): The new EE value to add to the chart.

        Returns:
            None
        """
        # Append the new data to the chart's series
        current_count = len(self.series1.points())
        self.series1.append(current_count, value)
        
        # # Adjust the x-axis range to show the latest data
        # if current_count > 10:  # For example, if we want to show only the last 10 data points
        #     self.axisX.setRange(current_count - 10, current_count)

    def update_chart_knob(self, data) -> None:
        """
        Update the chart with the reading knob force.

        Args:
            data: A ROS message containing the force data.

        Returns:
            None
        """
        # Convert ROS data to a suitable format for the chart
        value = data.force.data
        current_count = self.series2.count()
        self.series2.append(current_count, value)  
        if current_count > 100:  # For example, if we want to show only the last 100 data points
            self.axisX.setRange(current_count - 100, current_count)
            self.axisY.setRange(-10, 10) ## need to be change

    """
    UTILITIES
    """
    def check_current_selections(self) -> tuple:
        # Checking Mode
        if self.radioButton.isChecked():
            mode = "JOINT"
        elif self.radioButton_2.isChecked():
            mode = "TCP"
        else:
            mode = None
        
        # Checking TCP
        if self.radioButton_5.isChecked():
            tcp = 0
        elif self.radioButton_4.isChecked():
            tcp = 1
        elif self.radioButton_3.isChecked():
            tcp = 2
        else:
            tcp = None

        # Checking Joint
        joints = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        joint_radio_buttons = [self.radioButton_7, self.radioButton_8, self.radioButton_6, 
                            self.radioButton_9, self.radioButton_10, self.radioButton_11]
        joint = None
        for i, button in enumerate(joint_radio_buttons):
            if button.isChecked():
                joint = i
                break
        
        return mode, tcp, joint       

    def force_testing(self) -> None:
        # 10s of force testing
        for i in range(1000):
            knob_command = KnobCommand()
            knob_command.text.data = "force"
            knob_command.tcp_force.data = math.sin(rospy.Time.now().to_sec()) + 1.5
            self.knob_command_pub.publish(knob_command)
            rospy.sleep(0.01)

    """
    SERVICE CALLS
    """
    def move_arm_cartesian(self, pose) -> str:
        rospy.wait_for_service('move_arm_cartesian')
        x = pose[0]
        y = pose[1]
        z = pose[2]
        rx = pose[3]
        ry = pose[4]
        rz = pose[5]
        try:
            move_arm_cartesian = rospy.ServiceProxy('move_arm_cartesian', MoveArmCartesian)
            response = move_arm_cartesian(x, y, z, rx, ry, rz)
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed:", e)
            return False

    def move_arm_joint(self, joints) -> bool:
        rospy.wait_for_service('move_arm_joint')
        if len(joints) != 6:
            rospy.logerr("wrong joint nums")
            return False
        joint0 = joints[0]
        joint1 = joints[1]
        joint2 = joints[2]
        joint3 = joints[3]
        joint4 = joints[4]
        joint5 = joints[5]
        try:
            move_arm_joint = rospy.ServiceProxy('move_arm_joint', MoveArmJoint)
            response = move_arm_joint(joint0, joint1, joint2, joint3, joint4, joint5)
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed:", e)
            return False

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

    """
    PUBLICHERS
    """
    def publish_force(self, force=1.0) -> None:
        knob_command = KnobCommand()
        knob_command.text.data = "force"
        knob_command.tcp_force.data = float(force)
        self.knob_command_pub.publish(knob_command)

    def change_knob_state(self) -> None:
        # publish example
        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = 11
        knob_command.position.data = 0
        knob_command.position_width_radians.data = 10 * math.pi / 180
        knob_command.detent_strength_unit.data = 0.0
        knob_command.endstop_strength_unit.data = 1.0
        knob_command.snap_point.data = 1.1
        knob_command.text.data = "Bounded 0-10\nNo detents"
        self.knob_command_pub.publish(knob_command)

    def publish_knob_command(self) -> None:  
        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = int(self.num_positions_spinbox.value())
        knob_command.position.data = int(self.position_spinbox.value())
        knob_command.position_width_radians.data = float(self.position_width_radians_spinbox.value())
        knob_command.detent_strength_unit.data = float(self.detent_strength_unit_spinbox.value())
        knob_command.endstop_strength_unit.data = float(self.endstop_strength_unit_spinbox.value())
        knob_command.snap_point.data = float(self.snap_point_spinbox.value())
        knob_command.text.data = self.text_line_edit.text()
        knob_command.tcp_force.data = float(1.0)    
        self.knob_command_pub.publish(knob_command)

    """
    CALLBACKS 
    """
    def tcp_wrench_callback(self, data) -> None:    
        """
        tcp_wrench_callback
          force: 
            x: (-30, 30)
            y: (-30, 30)
            z: (-60, 30)

        """
        # add a threshold, if the force is larger than the threshold, then print the force
        CONTROL_MODE, TCP_AXIS, CONTROL_JOINT = self.check_current_selections()
        if CONTROL_MODE == "TCP":
            if TCP_AXIS == 0:
                current_force = data.wrench.force.x
                # clamp the force to (0, 3)
                clamp_force = max(1, min(abs(current_force)/10, 4))
            elif TCP_AXIS == 1:
                current_force = data.wrench.force.y
                # clamp the force to (0, 3)
                clamp_force = max(1, min(abs(current_force)/10, 4))
            else:
                current_force = data.wrench.force.z
                # clamp the force to (0, 3)
                clamp_force = max(1, min(abs(current_force)/10, 4))

            # self.update_chart_tcp(current_force)
            self.publish_force(clamp_force)

    def knob_state_callback(self, data) -> None: 
        if self.ur10_cart_cur is None:
            rospy.logwarn("Waiting for impedance controller start ...")
            rospy.sleep(1)
        elif self.ur10_cart_start is None:
            rospy.logwarn("Please push the mode publish button !")
            rospy.sleep(1)
        else:
            # if knob state changed, then send the command to the robot
            if self.knob_current_pos != data.position.data:
                CONTROL_MODE, TCP_AXIS, CONTROL_JOINT = self.check_current_selections()
                if CONTROL_MODE == "JOINT":
                    self.knob_current_pos = data.position.data
                    self.knob_current_force = data.force.data

                    joints = list(self.ur10_joint_start)
                    joints[CONTROL_JOINT] = joints[CONTROL_JOINT] + 0.1 * data.position.data
                    self.move_arm_joint(joints)
                    print("\r",joints, "                      ", end="")
                    return
                elif CONTROL_MODE == "TCP":
                    self.knob_current_pos = data.position.data
                    self.knob_current_force = data.force.data

                    position = self.ur10_cart_start.pose.position
                    position[int(TCP_AXIS)] = position[int(TCP_AXIS)] + 0.002 * self.knob_current_pos
                    print("\r",position, "                      ", end="")
                else:
                    rospy.logerr("Unknown mode: {}".format(CONTROL_MODE))
                    return

    def joint_state_callback(self, data) -> None:
        # update the joint state
        self.ur10_joint_cur = data.position
        self.doubleSpinBox_14.setValue(data.position[0])
        self.doubleSpinBox_15.setValue(data.position[1])
        self.doubleSpinBox_13.setValue(data.position[2])
        self.doubleSpinBox_16.setValue(data.position[3])
        self.doubleSpinBox_17.setValue(data.position[4])
        self.doubleSpinBox_18.setValue(data.position[5])
        rospy.sleep(0.4)
        
    def tcp_state_callback(self, data) -> None:
        self.ur10_cart_cur=data.pose
        self.text_line_edit.setValue(data.pose.position.x)
        self.doubleSpinBox_8.setValue(data.pose.position.y)
        self.doubleSpinBox_9.setValue(data.pose.position.z)
        rospy.sleep(0.4)

    def mode_change_callback(self, data) -> None:

        self.ur10_cart_start = self.ur10_cart_cur
        self.ur10_joint_start = self.ur10_joint_cur
        # TODO: reset KNOB CONTROL MODE



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()

    # Setup UI
    ui.setupUi(MainWindow)

    # Setup ROS
    ui.setup_ROS()

    MainWindow.show()
    # rospy.spin()
    
    sys.exit(app.exec_())