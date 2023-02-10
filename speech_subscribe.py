#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import subprocess

# run from beginning, wait for smoker detection module to trigger speech processing module
# once smoking activity detected, receive message from here and run fyp.sh
    
def speech_subscribe():
    rospy.init_node('speech_subscribe', anonymous=True)
    rospy.Subscriber("smoking_topic", String, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo("Input: %s", data.data)
    if data.data == "smoking":
        subprocess.call(["bash", "/home/mustar/catkin_ws/src/fyp_jiamun/fyp/fyp.sh"])

if __name__ == '__main__':
    speech_subscribe()
