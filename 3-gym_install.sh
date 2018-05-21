#!/bin/bash
# ROS_duckietown
# ROS + Dependencies Installation
# v 0.1


echo " ExBot ROS Gym    "
echo " By  Top Liu @ exbot.net    " 
echo " 2018.4.11"

echo " 
https://github.com/duckietown/gym-gazebo

"
:<<!
Ubuntu 16.04

Basic requirements:

    ROS Kinetic (/rosversion: 1.12.7)
    Gazebo 8.1.1
    Python 3.5.2 # I have yet to find out why though, because all the standard ROS stuff is in Python 2
    OpenCV3, installed from sources for Python 3 (git clone https://github.com/Itseez/opencv.git)
    OpenAI gym

ROS Kinetic dependencies
!

sudo pip2 install rospkg catkin_pkg
sudo pip2 install pyOpenSSL==16.2.0

sudo apt-get install python-pyqt4 \
libspnav-dev \
python-skimage \
pyqt4-dev-tools \
libcwiid-dev \ 
libbluetooth-dev -y

sudo apt-get install \
cmake gcc g++ qt4-qmake libqt4-dev \
libusb-dev libftdi-dev \
python-defusedxml python-vcstool \
ros-kinetic-octomap-msgs        \
ros-kinetic-joy                 \
ros-kinetic-geodesy             \
ros-kinetic-octomap-ros         \
ros-kinetic-control-toolbox     \
ros-kinetic-pluginlib	       \
ros-kinetic-trajectory-msgs     \
ros-kinetic-control-msgs	       \
ros-kinetic-std-srvs 	       \
ros-kinetic-nodelet	       \
ros-kinetic-urdf		       \
ros-kinetic-rviz		       \
ros-kinetic-kdl-conversions     \
ros-kinetic-eigen-conversions   \
ros-kinetic-tf2-sensor-msgs     \
ros-kinetic-pcl-ros \
ros-kinetic-navigation -y

#Install Sophus
mkdir ~/duckietown/gym-gazebo-dep
cd ~/duckietown/gym-gazebo-dep
git clone https://github.com/stonier/sophus -b indigo
cd sophus
mkdir build
cd build
cmake ..
make
sudo make install
echo "## Sophus installed ##\n"

##Gazebo gym
cd ~/duckietown
git clone https://github.com/erlerobot/gym-gazebo
cd gym-gazebo

#Build and install gym-gazebo
sudo pip2 install -e .
sudo pip3 install -e .

 
##Dependencies and libraries
sudo pip3 install h5py
sudo apt-get install python3-skimage -y

# install Theano
cd ~/duckietown/gym-gazebo-dep
git clone git://github.com/Theano/Theano.git
cd Theano/
sudo python3 setup.py develop

#install Keras
sudo pip3 install keras

 
 

# make sure to switch to bash, because the installation script 
# hasn't been converted to ZSH yet
# and if you run it from ZSH the environmental variables 
# that are set during the script's execution aren't preserved
rm ~/duckietown/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/ecl_navigation


bash
cd ~/duckietown/gym-gazebo/gym_gazebo/envs/installation
./setup_kinetic.bash	

sudo apt-get install ros-kinetic-turtlebot-* -y

cd ~/duckietown/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src && catkin_init_workspace
  cd ..
  source devel/setup.bash
	catkin build

cd ~/duckietown/gym-gazebo/gym_gazebo/envs/installation
./turtlebot_setup.bash

#Run the environment with a sample agent:

# only execute this while still in the same bash as last step
cd ~/duckietown/gym-gazebo/examples/turtlebot
python circuit2_turtlebot_lidar_qlearn.py

#We recommend creating an alias to kill those processes.

#echo "alias killgym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc

 
	



