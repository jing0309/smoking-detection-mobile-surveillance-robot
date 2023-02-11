#!/usr/bin/env python

"""

	- roslaunch jupiterobot_bringup jupiterobot_bringup.launch
	- YOLO file
	- rosrun navigation_module take_photo.py
	- rosrun navigation_module distance_prediction.py
	- rosrun navigation_module scanning.py

"""

import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import radians

class Scan:
	def __init__(self):
		while not rospy.is_shutdown():
			rospy.loginfo("Scan")

			# ================== INITIALIZATION ================== 
			# Initialize cmd_vel for robot rotation
			self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

			# Initialize scanning result publisher
			self.point_done = rospy.Publisher('point_done', String, queue_size=10)

			# Initialize start distance prediction publisher
			self.start_distance_prediction = rospy.Publisher('start_distance_prediction', String, queue_size=10)
			# ==================================================== 

			# Wait for scanning command
			self.rotate_record = 0 
			rospy.wait_for_message('start_scanning', String)
			# If the robot turn less than 360 degree
			while self.rotate_record <= 360:
				rospy.loginfo("Turning")
				r = rospy.Rate(5)

				turn_cmd = Twist()
				turn_cmd.linear.x = 0
				turn_cmd.angular.z = radians(15) # turn 15 degree

				for x in range(0,10):
					self.cmd_vel.publish(turn_cmd)
					r.sleep()
				
				self.rotate_record += 15
				
				# Wait message from person_detection
				self.person_detected_indicator = rospy.wait_for_message('person', String)
				# If person detected, end the loop and start distance prediction
				if self.person_detected_indicator.data == "Person detected":
					self.start_distance_prediction.publish('done')
					break
				
			rospy.loginfo("End Scanning")
			# If no person detected, publish point done message 
			if self.rotate_record >= 360:
				self.point_done.publish('done')

if __name__=="__main__":
	rospy.init_node('scanning')
	try:
		Scan()
	except:
		pass
