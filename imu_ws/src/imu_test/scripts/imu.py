#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
 * Copyright (c) 2021, Micropolis
 * All rights reserved.
 
 * Created on Sun Aug 29 10:30:17 2021
 * @author: Khaled Gabr
 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  IMU Node SUBSCRIBER test app
 *
 *  Copyright 2021  Micropolis Team
 *  http://www.micropolis.ae
 * 
 */
 """
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
