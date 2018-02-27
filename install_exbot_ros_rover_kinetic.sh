#!/bin/bash
# ROS_Rover
# ROS + Dependencies Installation
# v 0.1
echo -e "\e[1m \e[34m >>>   ExBot ROS Rover   <<<"

echo " By ChaoZheng Zhu, Top Liu    " 
echo " 2018.2.11"

echo -e "\e[1m \e[34m >>> The default installation path is ~/catkin_ws \e[21m \e[39m"
echo -e "\e[1m \e[34m >>> Beginning ROS Kinetic Installation \e[21m \e[39m"
echo -e "\e[34m >>> Setting up sources.list and keys... \e[39m"

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

echo -e "\e[34m >>> ...done\e[39m"

  sudo apt-get update

echo -e "\e[34m >>> Beginning ros-kinetic-desktop-full installation...\e[39m"

  sudo apt-get --yes  install ros-kinetic-desktop-full
  

echo -e "\e[34m >>> Setting up rosdep\e[39m"

  sudo rosdep init
  rosdep update

echo -e "\e[34m >>> Setting up environment \e[39m"

  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc

echo -e "\e[34m >>> Setting up rosinstall \e[39m"

  sudo apt-get --yes --force-yes install python-rosinstall


  
echo -e "\e[1m \e[34m >>> Installing support software \e[21m \e[39m"

  sudo apt-get --yes --force-yes install git
  sudo add-apt-repository -y ppa:webupd8team/sublime-text-3
  sudo apt-get update
  sudo apt-get --yes --force-yes install sublime-text-installer 

  sudo apt-get --yes --echo -e "\e[34m >>> Setting up rosinstall \e[39m"

  sudo apt-get --yes --force-yes install python-rosinstall install netbeans
  sudo apt-get --yes --force-yes install gitk git-gui
  sudo apt-get --yes --force-yes install kazam vlc
  
  
 #### Installing dependencies ############################################################################################ 
echo -e "\e[1m \e[34m >>> Installing dependencies for mobile robotics code \e[21m \e[39m"

sudo apt-get --yes --force-yes install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control 
sudo apt-get --yes --force-yes install ros-kinetic-multimaster-launch 
# Next you will need to install the Kinetic packages for the simulated LIDAR.
sudo apt-get --yes --force-yes install ros-kinetic-lms1xx
# Also I installed the below somehow but they might not be needed actually.
sudo apt-get --yes --force-yes install hector-gazebo
sudo apt-get --yes --force-yes install ros-kinetic-interactive-marker-twist-server 
sudo apt-get --yes --force-yes install ros-kinetic-twist-mux
sudo apt-get --yes --force-yes install ros-kinetic-imu-tools 
  source ~/.bashrc
  
 #### Setting up workspace ######################################################################################################## 
 echo "##### Setting up workspace. ####################################"

  source /opt/ros/kinetic/setup.bash
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  rosdep update

  mkdir -p ~/catkin_ws/src
	
  cd ~/catkin_ws/src  && catkin_init_workspace
  
  
  #### git #############################################################################################################
  git config --global user.name "$USERNAME"
  git config --global user.email "$EMAIL"
 
  cd ~/catkin_ws/src && git clone https://github.com/husky/husky.git  -b kinetic-devel
  cd ~/catkin_ws/src && git clone https://github.com/BlueWhaleRobot/nav_test.git
  

#### make #######################################################
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  cd ~/catkin_ws && rosdep install --from-path src --ignore-src 
  cd ~/catkin_ws && catkin_make
  cd ~/catkin_ws && catkin_make install

  # set the environment variable HUSKY_GAZEBO_DESCRIPTION

  echo "export HUSKY_URDF_EXTRAS=$(rospack find husky_description)/urdf/empty.urdf" >> ~/.bashrc
  
  echo "export ROS_WORKSPACE=$HOME'/catkin_ws'" >> ~/.bashrc
  echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
  
  # echo "export ROS_IP=`ifconfig eth0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://'`" >> ~/.bashrc
  # echo "ROS_IP=`ifconfig eth0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://'`"




echo "[!!!] NB: You must still manually add your ROS_IP to your ~/.bashrc.  Do this by checking your IP with hostname -I or ifconfig and then adding export ROS_IP='x.x.x.x' to your ~/.bashrc." 
  
  
source ~/catkin_ws/devel/setup.bash




