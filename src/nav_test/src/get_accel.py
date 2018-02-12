#!/usr/bin/env python
# coding: UTF-8

'''
计算最大加速度
'''

import rospy
from nav_msgs.msg import Odometry
import math
import signal
import sys

rospy.init_node('accel_cal', anonymous=True)
previousodom = None
accelDataFile = open("accel.txt", "w+")
fileData = ""

def cal_accel(odom):
    global previousodom
    global accelDataFile
    global fileData
    if previousodom == None:
        previousodom = odom
        return
    delta_vx = odom.twist.twist.linear.x - previousodom.twist.twist.linear.x
    delta_vy = odom.twist.twist.linear.y - previousodom.twist.twist.linear.y
    delta_v = math.sqrt( delta_vx * delta_vx + delta_vy * delta_vy )
    delta_t = odom.header.stamp.to_sec() - previousodom.header.stamp.to_sec()
    accel = delta_v / delta_t
    #if accel < 0.01:
    #    previousodom = odom
    #    return
    print accel
    fileData += str(accel) + "\n"
    previousodom = odom

def signal_handler(signal, frame):
    print "saving data"
    print fileData
    accelDataFile.write(fileData)
    accelDataFile.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

rospy.Subscriber("/xqserial_server/Odom", Odometry , cal_accel)
rospy.spin()
