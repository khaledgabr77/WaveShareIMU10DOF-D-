#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from sensor_msgs.msg import Imu


def print_state(data):
    print("\n\n\n\n")
    rospy.loginfo("IMU Data : ")
    rospy.loginfo(data)
    print("\n\n\n\n")

if __name__=="__main__":
    rospy.init_node('imu_listener',anonymous=True)
    rospy.Subscriber('/imu_data',Imu,print_state)
    rospy.loginfo("'imu_listener' node is listening IMU messages from Terminal!")
    rospy.spin() 
