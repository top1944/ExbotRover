from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import rospy

pub = rospy.Publisher('/odom', String, queue_size=10)
rospy.init_node('get_odom', anonymous=True)

def get_odom(message):
    odom = Odometry()
    current_time = rospy.Time.now()
    odom.header.stamp = current_time
    odom.frame_id = "odom"
    odom.pose.pose.position.x =
    odom.pose.pose.position.y =
    odom.pose.pose.position.z =
    odom.pose.pose.orientation = ;

def start():
    rospy.Subscriber("chatter", String, get_odom)
    rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == "__main__":
