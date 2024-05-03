# Drone Racing Project

Simple ROS2 simulation + controller making a tello drone fly through frames and park on a stop sign

## Demo

- Video available for download in [https://seafile.utu.fi/f/a3fa8195b8fb4b378d2b/](https://seafile.utu.fi/f/a3fa8195b8fb4b378d2b/)

## Running (Ubuntu 20.04)

- Clone this project
- Have the prerequisites installed (ROS2 Galactic, ros-cv-bridge, requirements.txt)
- `./source_build_run_all.sh` or call the launch file directly

## Running (non Ubuntu systems)

- Have docker installed on a X11 enabled system (linux, mac does not have GPU accel yet)
- If you are part of the docker group: `./start.sh`, else `sudo ./start.sh`
- Have fun!
