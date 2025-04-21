#!/bin/bash
sleep 4
source /opt/ros/noetic/setup.bash

source /home/right/livox/devel/setup.bash

gnome-terminal -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 4

gnome-terminal -- bash -c "roslaunch point_lio mapping_horizon.launch; exec bash" 
sleep 2

source /home/right/Position/devel/setup.bash
gnome-terminal -- bash -c "roslaunch position position.launch; exec bash" 
sleep 2

echo "ROS launch file started successfully!"