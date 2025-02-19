#!/bin/bash
sleep 5
source /opt/ros/noetic/setup.bash

source /home/right/Desktop/Livox/devel/setup.bash

gnome-terminal -- bash -c "sudo chmod 666 /dev/ttyUSB0; exec bash"
sleep 3

gnome-terminal -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 3

gnome-terminal -- bash -c "roslaunch point_lio mapping_horizon.launch; exec bash" 
sleep 3

source /home/right/Position/devel/setup.bash
gnome-terminal -- bash -c "rosrun position position; exec bash" 
sleep 3

echo "ROS launch file started successfully!"
