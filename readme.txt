-------------------------------------------------------------------------------

curiosity_mars_rover_ws package
Uses ROS Noetic & Python 3, tested on Ubuntu 20.04

-------------------------------------------------------------------------------

To install everything automatically:
Make sure this folder (curiosity_mars_rover_ws) is in your /home/username/ folder (where 'username' is your Linux user name).
In a terminal, navigate to this folder (curiosity_mars_rover_ws) and type:

  bash install_everything.sh

The installation will take roughly 10 minutes and use 4.8GB of disk space.
Afterwards, to reload your current terminal window run:

  exec bash

You will now be able to run 'roslaunch' commands, for example:

  roslaunch curiosity_mars_rover_gazebo main_mars_terrain.launch



-------------------------------------------------------------------------------

Additional notes

-------------------------------------------------------------------------------

The install_everything.sh script can also take an alternative workspace folder name as a parameter.
For instance, if you have renamed this 'curiosity_mars_rover_ws' folder to 'catkin_ws' (the typical ROS workspace name):

  bash install_everything.sh catkin_ws

This will install everything with the workspace 'catkin_ws'.

-------------------------------------------------------------------------------

The web application requires a security certificate in order to run. An example has been provided and will be installed as part of the installation script, however this certificate will expire at some point in the future. If you have discovered this repository amid the ruins of civilisation, new certificates can be created using the following command:

  openssl req -x509 -newkey rsa:4096 -keyout server1.example.com.key -out server1.example.com.pem -days 365 -nodes

The two new files should then be moved to the system configuration folder:

  sudo mkdir /etc/ssl/certs/localcerts/ -p
  sudo mv server1.example.com.key /etc/ssl/certs/localcerts/server1.example.com.key
  sudo mv server1.example.com.pem /etc/ssl/certs/localcerts/server1.example.com.pem

Finally, the permissions of the files should be set:

  sudo chmod 777 /etc/ssl/certs/localcerts/server1.example.com.key
  sudo chmod 777 /etc/ssl/certs/localcerts/server1.example.com.pem
