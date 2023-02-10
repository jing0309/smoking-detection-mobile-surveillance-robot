#!/bin/bash


# roscore &
rosrun fyp question.py
python3 /home/mustar/catkin_ws/src/fyp_jiamun/fyp/rnn.py
python3 /home/mustar/catkin_ws/src/fyp_jiamun/fyp/report_generation.py
rosrun fyp speech_publisher.py
