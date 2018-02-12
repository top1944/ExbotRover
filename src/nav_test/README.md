# nav_test
some tools in remote control and navigation of xiaoqiang. The function of each tools is written in each source code file.
## Usage:
### 1.download to xiaoqiang ros workspace
```
cd ~/Documents/ros/src
git clone https://github.com/BlueWhaleRobot/nav_test.git 
cd ..
catkin_make
```
#### 2.remote control xiaoqiang by keboard direction keys (use space key to stop it) 
```
ssh xiaoqiang@192.168.x.x     #please change 192.168.x.x to a real ip address  
rosrun nav_test control.py
```
## Made with :heart: by BlueWhale Tech corp.
    
    
一些用于远程遥控和导航的工具程序。每个工具程序的作用都写在了相应程序的源代码里面。
          
## 使用方法：
### 1.安装到小强ROS工作目录
```
cd ~/Documents/ros/src
git clone https://github.com/BlueWhaleRobot/nav_test.git 
cd ..
catkin_make
```
### 2.用键盘方向键远程控制小强移动（留意空格是停止）    
```
ssh xiaoqiang@192.168.x.x     #请将192.168.x.x换成实际的ip
rosrun nav_test control.py
```
## 由蓝鲸科技精 :heart: 制作。
