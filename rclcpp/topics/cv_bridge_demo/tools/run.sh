#!/bin/bash

export ROS_HOME=~/.ros 
export ROS_LOG_DIR=/home/hugoliu/github/colcon_ws/examples/log 
export PRJ_PATH=/home/hugoliu/github/colcon_ws/examples/
source ${PRJ_PATH}/install/setup.bash
ros2 launch cv_bridge_demo demo_launch.py