#!/bin/bash

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export GAZEBO_PLUGIN_PATH=GAZEBO_PLUGIN_PATH:/home/steve/Project/DEV/catkin_ws/build_isolated/myriad_gazebo_plugin/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo


export PX4_HOME_LAT=44.805261
export PX4_HOME_LON=-0.605788
export PX4_HOME_ALT=0.0
export PX4_GPS_FIX_TYPE=6
export PX4_GPS_NB_SAT=20

pkill -9 gzserver
pkill -9 gzclient
roslaunch px4 swarm5.launch gui:=true
