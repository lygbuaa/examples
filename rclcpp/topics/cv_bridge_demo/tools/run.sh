#!/bin/bash

export ROS_HOME=~/.ros
# set ros2-log-dir
export ROS_LOG_DIR=/home/hugoliu/github/colcon_ws/examples/log 
# set ros2-log-format, goto https://docs.ros.org/en/foxy/Tutorials/Demos/Logging-and-logger-configuration.html
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{file_name}:{line_number}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1

# setup env
export PRJ_PATH=/home/hugoliu/github/colcon_ws/examples/
source ${PRJ_PATH}/install/setup.bash

ros2 launch cv_bridge_demo demo_launch.py