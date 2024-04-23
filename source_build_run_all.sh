#!/bin/bash

cd ~/drone_racing_ws/

source /opt/ros/galactic/setup.bash
source install/setup.bash

export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh

colcon build
ros2 launch tello_process launch.py
