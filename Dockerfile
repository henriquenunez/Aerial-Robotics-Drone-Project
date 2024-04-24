FROM osrf/ros:galactic-desktop

RUN sudo apt update
RUN sudo apt install -y gazebo11 libgazebo11 libgazebo11-dev
RUN sudo apt install -y libasio-dev
RUN sudo apt install -y ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers 
RUN sudo apt install -y ros-galactic-gazebo-ros-pkgs
RUN sudo apt install -y libignition-rendering3 
RUN sudo apt install -y python3-pip
RUN sudo apt install -y mesa-utils 
RUN sudo apt install -y git 
RUN pip3 install transformations
RUN pip3 install filterpy 

RUN useradd -m rosusr
USER rosusr

WORKDIR /home/rosusr
COPY scripts/prepare_workspace.sh . 
COPY scripts/run_everything.sh . 
COPY src/tello_process ./drone_racing_ws/src/tello_process
#COPY src/tello_gazebo ./drone_racing_ws/src/tello_gazebo
COPY src/tello_ros ./drone_racing_ws/src/tello_ros
COPY src/ros2_shared ./drone_racing_ws/src/ros2_shared

USER root
WORKDIR /home/rosusr/drone_racing_ws
RUN chmod 777 -R .

USER rosusr
WORKDIR /home/rosusr
RUN ./prepare_workspace.sh
#RUN ./drone_racing_ws/src/example_cpp_pkg/build.sh

