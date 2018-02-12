#!/usr/bin/env python
# coding: UTF-8
import roslib
import rospy
from geometry_msgs.msg import  Twist
from math import radians, pi
from std_msgs.msg import Bool
class BarDetectChanger:

    def __init__(self):
        rospy.init_node('BarDetectChanger', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        bar_disabled = rospy.get_param("~barDetectFlag", 1.0)

        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.barDetectFlag_pub = rospy.Publisher('/barDetectFlag', Bool, queue_size=1)
        if bar_disabled == 1 :
            return
        rospy.loginfo("Stopping the robot...")
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())

        rospy.loginfo("disable BarDetectFlag")
        r = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            flag=Bool()
            flag.data=False
            self.barDetectFlag_pub.publish(flag) # disable BarDetectFlag
            r.sleep()


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("enable BarDetectFlag")
        # enable BarDetectFlag
        flag=Bool()
        flag.data=True
        self.barDetectFlag_pub.publish(flag)
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        BarDetectChanger()
    except rospy.ROSInterruptException:
        rospy.loginfo("BarDetectChanger finished.")
