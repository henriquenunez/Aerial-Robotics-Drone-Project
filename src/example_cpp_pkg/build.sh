#!/bin/bash

cd ~/drone_racing_ws/ 

# Source ROS2
source /opt/ros/galactic/setup.bash
colcon build --packages-skip drone_project example_cpp_pkg

if test -d install; then
  echo "Directory exists."
  source install/setup.bash
  colcon build
fi


