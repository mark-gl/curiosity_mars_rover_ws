#! /bin/bash
default_ws='curiosity_mars_rover_ws'
CATKIN_WORKSPACE=${1:-$default_ws}

sudo apt update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y python3 ros-noetic-desktop-full
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-pytest build-essential
sudo apt install -y ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-amcl ros-noetic-gmapping ros-noetic-map-server ros-noetic-move-base ros-noetic-rtabmap ros-noetic-rtabmap-ros ros-noetic-ira-laser-tools ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server
sudo apt install -y hugin-tools imagemagick-6.q16 enblend

echo "" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/$CATKIN_WORKSPACE/devel/setup.bash" >> ~/.bashrc
chmod +x ~/$CATKIN_WORKSPACE/src/curiosity_mars_rover_control/scripts/*
chmod +x ~/$CATKIN_WORKSPACE/src/curiosity_mars_rover_navigation/scripts/*
chmod +x ~/$CATKIN_WORKSPACE/src/curiosity_mars_rover_viz/scripts/*

sudo mkdir /etc/ssl/certs/localcerts/ -p
sudo cp ~/$CATKIN_WORKSPACE/src/curiosity_mars_rover_viz/certs/server1.example.com.key /etc/ssl/certs/localcerts/server1.example.com.key
sudo cp ~/$CATKIN_WORKSPACE/src/curiosity_mars_rover_viz/certs/server1.example.com.pem /etc/ssl/certs/localcerts/server1.example.com.pem
sudo chmod 777 /etc/ssl/certs/localcerts/
sudo chmod 777 /etc/ssl/certs/localcerts/server1.example.com.key
sudo chmod 777 /etc/ssl/certs/localcerts/server1.example.com.pem
sudo ufw allow 9090
sudo ufw allow 8080

. /opt/ros/noetic/setup.bash
cd ~/$CATKIN_WORKSPACE/
catkin_make
. devel/setup.bash

echo "---"
echo "All done! Open a new terminal to start using ROS, Gazebo and the Curiosity rover simulation."
