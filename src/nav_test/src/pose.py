#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

rospy.init_node('pose_talker', anonymous=True)
vis_pub = rospy.Publisher('/cur_pose', Marker, queue_size=0)
marker = Marker()
marker.header.frame_id = "map";
marker.header.stamp = rospy.Time.now();
marker.ns = "my_namespace";
marker.id = 0;
marker.type = Marker.ARROW;
marker.action = Marker.MODIFY;
marker.pose.position.x = 1;
marker.pose.position.y = 1;
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.2;
marker.scale.y = 0.03;
marker.scale.z = 0.1;
marker.color.a = 1.0;
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

def pub_pose(odom):
    marker.pose.position.x = odom.pose.pose.position.x
    marker.pose.position.y = odom.pose.pose.position.y
    marker.pose.orientation.x = odom.pose.pose.orientation.x
    marker.pose.orientation.y = odom.pose.pose.orientation.y
    marker.pose.orientation.z = odom.pose.pose.orientation.z
    marker.pose.orientation.w = odom.pose.pose.orientation.w
    vis_pub.publish( marker );

rospy.Subscriber("/xqserial_server/Odom", Odometry , pub_pose)
rospy.spin()
