#!/bin/bash

if [[ "$1" = "build" ]]; then
  docker build -t ros_container .
fi

if [[ "$1" = "build-no-cache" ]]; then
  docker build --no-cache -t ros_container .
fi

xhost +

# For intel GPU
docker run -it \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --env="DISPLAY=$DISPLAY" \
  ros_container \
  bash #drone_racing_ws/src/drone_project/run_everything.sh

