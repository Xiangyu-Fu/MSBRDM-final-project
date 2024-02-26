#!/usr/bin/env python3
# Author: Xuhao Jin
# Date: 2024-02-25
# Description: This script is used to provid fake force.


import rospy
from geometry_msgs.msg import WrenchStamped

rospy.init_node('fake_wrench_publisher')
pub = rospy.Publisher('/fake_schunk_netbox/raw', WrenchStamped, queue_size=10)

rate = rospy.Rate(10)  # 10hz frequency to publish the messages

pulse = False  # Boolean to toggle between two states

while not rospy.is_shutdown():
    # Creating a new WrenchStamped message
    wrench_msg = WrenchStamped()
    wrench_msg.header.stamp = rospy.Time.now()
    wrench_msg.header.frame_id = "ft_sensor_link"

    if pulse:
        # Assigning one set of force and torque data
        wrench_msg.wrench.force.x = -131.479277
        wrench_msg.wrench.force.y = -76.275814
        wrench_msg.wrench.force.z = 485.809591
        wrench_msg.wrench.torque.x = -0.78713
        wrench_msg.wrench.torque.y = -6.932118
        wrench_msg.wrench.torque.z = -4.1215
    else:
        # Assigning an alternative set of force and torque data for the pulse effect
        wrench_msg.wrench.force.x = 0
        wrench_msg.wrench.force.y = 0
        wrench_msg.wrench.force.z = 0
        wrench_msg.wrench.torque.x = 0
        wrench_msg.wrench.torque.y = 0
        wrench_msg.wrench.torque.z = 0

    # Toggle the pulse boolean
    pulse = not pulse

    # Publishing the message
    pub.publish(wrench_msg)
    
    # Sleeping to maintain the publishing rate
    rate.sleep()