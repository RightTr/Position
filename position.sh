#!/bin/bash
sleep 4
source /opt/ros/noetic/setup.bash

source /home/nf/Desktop/Livox/devel/setup.bash

gnome-terminal -- bash -c "sudo chmod +666 /dev/ttyUSB0; exec bash"
sleep 2

gnome-terminal -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 2

gnome-terminal -- bash -c "roslaunch point_lio mapping_horizon.launch; exec bash" 
sleep 2

source /home/nf/depth_clustering_ws/devel/setup.bash
gnome-terminal -- bash -c "roslaunch depth_clustering Mid360.launch; exec bash"
sleep 2

source /home/nf/Position_dc_jetson/devel/setup.bash
gnome-terminal -- bash -c "rosrun position position; exec bash" 
sleep 2

echo "ROS launch file started successfully!"
