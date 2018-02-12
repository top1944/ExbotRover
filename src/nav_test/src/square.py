#!/usr/bin/env python
# coding: UTF-8
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveBaseSquare:

    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # How big is the square we want the robot to navigate?
        # 设定正方形的尺寸，默认是一米
        square_size = rospy.get_param("~square_size", 1.0) # meters

        # Create a list to hold the target quaternions (orientations)
        # 创建一个列表，保存目标的角度数据
        quaternions = list()

        # First define the corner orientations as Euler angles
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)
        euler_angles = (pi/2, pi, 3*pi/2, 0)

        # Then convert the angles to quaternions
        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        # 创建一个列表存储导航点的位置
        waypoints = list()

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        # 创建四个导航点的位置（角度和坐标位置）
        waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[0]))
        waypoints.append(Pose(Point(square_size, square_size, 0.0), quaternions[1]))
        waypoints.append(Pose(Point(0.0, square_size, 0.0), quaternions[2]))
        waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))

        # Initialize the visualization markers for RViz
        # 初始化可视化标记
        self.init_markers()

        # Set a visualization marker at each waypoint
        # 给每个定点的导航点一个可视化标记(就是rviz中看到的粉色圆盘标记)
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)

        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)

        # Subscribe to the move_base action server
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        # Initialize a counter to track waypoints
        # 初始化一个计数器，记录到达的顶点号
        i = 0

        # Cycle through the four waypoints
        # 主循环,环绕通过四个定点
        while i < 4 and not rospy.is_shutdown():
            # Update the marker display
            # 发布标记指示四个目标的位置，每个周期发布一起，确保标记可见
            self.marker_pub.publish(self.markers)

            # Intialize the waypoint goal
            # 初始化goal为MoveBaseGoal类型
            goal = MoveBaseGoal()

            # Use the map frame to define goal poses
            # 使用map的frame定义goal的frame id
            goal.target_pose.header.frame_id = 'map'

            # Set the time stamp to "now"
            # 设置时间戳
            goal.target_pose.header.stamp = rospy.Time.now()

            # Set the goal pose to the i-th waypoint
            # 设置目标位置是当前第几个导航点
            goal.target_pose.pose = waypoints[i]

            # Start the robot moving toward the goal
            # 机器人移动
            self.move(goal)

            i += 1

    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            # 把目标位置发送给MoveBaseAction的服务器
            self.move_base.send_goal(goal)

            # Allow 1 minute to get there
            # 设定1分钟的时间限制
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

            # If we don't get there in time, abort the goal
            # 如果一分钟之内没有到达，放弃目标
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")

    def init_markers(self):
        # Set up our waypoint markers
        # 设置标记的尺寸
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        # 定义一个标记的发布者
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=0)

        # Initialize the marker points list.
        # 初始化标记点的列表
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
