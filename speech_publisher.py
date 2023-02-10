#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def speech_publish():
    rospy.init_node('speech_publish', anonymous=True)
    pub = rospy.Publisher('point_done', String, queue_size=10)
    rospy.sleep(5)
    pub.publish("done")
        

if __name__ == '__main__':
    try:
        speech_publish()
    except rospy.ROSInterruptException:
        pass

