#!/bin/bash

cd ~/drone_racing_ws/

# Source ROS2
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash

# Source gazebo
export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh

# cd src/

# ros2 launch tello_gazebo demo_track_launch.py

#ros2 launch src/drone_project/launch/demo_track_launch.py 
ros2 launch src/example_cpp_pkg/launch/demo_track_launch.py 


# ros2 launch drone_project/launch/demo_track_l aunch.py 

# ros2 launch demo_track_launch.py 
# ros2 launch drone_project demo_track_launch.py
# cd drone_project/
# cd ~/drone_racing_ros2_ws/src

# ros2 launch drone_project/launch/demo_track_launch.py 
# ros2 topic echo /drone1/image_raw 
# ros2 run image_view image_view 
# ros2 run image_view image_view image:=/drone1/image_raw
# source /opt/ros/galactic/setup.bash
# source install/setup.bash 
# ros2 run image_view image_view image:=/drone1/image_raw
# ros2 run image_view image_view --ros-args --remap image:=/drone1/image_raw
# ros2 topic hz /drone1/image_raw 
# ros2 topic echo /drone1/image_raw 
# rviz2 & >> /dev/null
