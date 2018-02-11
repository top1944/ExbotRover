# Exbot ROS Rover  
版本Kinetic

Chazozheng Zhu
Top Liu

软件：ROS适用于四驱智能车
车体：巨匠小路虎
车载主控：树莓派Ubuntu mate 16.04 + Kinetic
控制站PC： Ubuntu 16.04 + Kinetic

![](http://leemanchina.com/robot/imagesserver/uploadpic/20171213172825122.jpg)
![](http://robohub.org/wp-content/uploads/2014/03/Screenshot-from-2014-03-14-07_34_30-1024x532.png)

## ROS机器人使用手册 ####
1. 连接树莓派到本地的局域网
将树莓派取出， 通过HDMI线连接到电脑屏幕， 通过鼠标选择当前的网络， 进行连接
进入路由器找到当前的无线网中， 树莓派所属的IP地址
配置下hosts文件， 便于实现分布式通信
vi /ect/hosts
添加下面一行
(真实ubuntu主机的ip) （真实Ubuntu主机的名字）
例如
192.168.1.102 blackant-desktop
####2. 启动树莓派中的串口server程序
首先我们先给USB串口一个权限
sudo chmod 777 /dev/ttyUSB0
我们将连接到树莓派 进入/home/black/catkin_ws/src/serial_server/build/devel/lib/serial_server/目录， 启动serial_server文件
####3. 在真实ubuntu主机上启动模型 首先我们需要安装husky模型， 在这里必须要确保你的ROS系统是indigo， 目前husky只支持indigo， 所以我们在Indigo安装如下
sudo apt-get install ros-indigo-husky-desktop
sudo apt-get install ros-indigo-husky-simulator
之后可以进行编译了
catkin_make
然后这个时候运行下面的命令， 很可能报错， 我们可以通过重启电脑解决
roslaunch husky_description description.launch
当我们搞好了第一步模型之后呢， 不能只能使用， 最好是新建一个fake.launch文件
<?xml version="1.0"?>
<launch>
<arg name="laser_enabled" default="$(optenv HUSKY_LMS1XX_ENABLED false)"/>
<arg name="ur5_enabled" default="$(optenv HUSKY_UR5_ENABLED false)"/>
<arg name="kinect_enabled" default="false"/>
<!-- Standalone launcher to visualize the robot model. -->

  <include file="$(find husky_gazebo)/launch/spawn_husky.launch">
    <arg name="laser_enabled" value="$(arg laser_enabled)"/>
    <arg name="kinect_enabled" value="$(arg kinect_enabled)"/>
  </include>


<include file="$(find husky_description)/launch/description.launch">
<arg name="laser_enabled" value="$(arg laser_enabled)"/>
<arg name="ur5_enabled" value="$(arg ur5_enabled)"/>
<arg name="kinect_enabled" value="$(arg kinect_enabled)"/>
</include>
<arg
name="gui"
default="True" />
<param
name="use_gui"
value="$(arg gui)" />
<node
name="joint_state_publisher"
pkg="joint_state_publisher"
type="joint_state_publisher" />
<!-- <node
name="robot_state_publisher"
pkg="robot_state_publisher"
type="state_publisher" /> -->
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find husky_viz)/rviz/model.rviz" />
</launch>
####4. 控制Husky跑起来
推荐下载一个小工具nav_test到ubuntu主机的catkin_ws/src目录下
git clone https://github.com/BlueWhaleRobot/nav_test.git
这个时候再进行一次编译
caktin_make
启动control.py就可以实现通过按键对机器人进行控制
rosrun nav_test control.py
####5. 完整的运行 树莓派端
roscore &
sudo chmod 777 /dev/ttyUSB0
最好是到目录下启动serial_server
cd ~/catkin_ws/devel/lib/serial_server & ./serial_server
./serial_server
主机端 添加树莓派网络名和IP对应（同上）
vi /etc/hosts
192.168.1.105 ubuntu(根据实际情况进行修改)
export ROS_MASTER_URI=http://(树莓派的网络名字):11311
