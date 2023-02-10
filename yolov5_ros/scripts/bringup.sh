#!/bin/bash
roslaunch usb_cam usb_cam-test.launch &
sleep 3 &
roslaunch jupiterobot_bringup jupiterobot_bringup.launch &
sleep 3 &
roslaunch jupiterobot_navigation rplidar_amcl_demo.launch map_file:=/home/mustar/catkin_ws/src/jinghui_fyp_project/jinghui_navigation/maps/lab_map.yaml &
sleep 3 &
roslaunch turtlebot_rviz_launchers view_navigation.launch