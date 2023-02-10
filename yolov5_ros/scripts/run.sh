#!/bin/bash

roslaunch yolov5_ros default_yolo.launch &
roslaunch jinghui_navigation navigation.launch &
roslaunch yolov5_ros smoking_detection.launch & 
rosrun fyp speech_subscribe.py