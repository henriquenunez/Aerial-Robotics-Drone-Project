#!/bin/bash


colcon build --packages-select tello_process
. install/setup.bash 
 ros2 launch tello_process launch.py 



