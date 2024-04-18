#!/bin/bash

cd ~/drone_racing_ros2_ws/ 

# Source ROS2
source /opt/ros/galactic/setup.bash
source install/setup.bash

# Source gazebo
export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models

source /usr/share/gazebo/setup.sh

## Starting gazebo
#WORLD_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/worlds/demo_track_edit.world
#gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so $WORLD_PATH & 
##

#echo "Sleeping for 4 seconds!"
#sleep 4s
#echo "waking up xd"

ros2 launch example_cpp_pkg demo_track_launch.py 
#ros2 launch tello_gazebo simple_launch.py 

