#!/usr/bin/env python

"""

	- roslaunch jupiterobot_bringup jupiterobot_bringup.launch
	- roslaunch jupiterobot_navigation rplidar_amcl_demo.launch map_file:=/home/mustar/catkin_ws/src/jinghui_fyp_project/navigation_module/maps/lab_map.yaml
	- roslaunch turtlebot_rviz_launchers view_navigation.launch
	- rosrun navigation_module smoking_patrol.py

"""

import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from math import radians

original = 0

class officeTour:
	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		# ================== INITIALIZATION ================== 
		# Subscribe to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.loginfo("Waiting for move_base action server...")

		# Wait for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(120))
		rospy.loginfo("Connected to move base server")

		# A variable to hold the initial pose of the robot to be set by the user in RViz
		initial_pose = PoseWithCovarianceStamped()
		rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

		# Get the initial pose from the user
		rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
		rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

		# Initialize start_scanning publisher for scanning and person detection
		self.start_scanning = rospy.Publisher('start_scanning', String, queue_size=10)

		# Make sure we have the initial pose
		while initial_pose.header.stamp == "":
			rospy.sleep(1)
		# =====================================================
			
		rospy.loginfo("Ready to go")
		rospy.sleep(1)

		# Coordinate for 4 corner
		locations = dict()
		locations['Corner 1'] = [1.5923, -1.2323, 0.4412, 0.8974]		
		locations['Corner 2'] = [2.1519, -3.1044, -0.3279, 0.9447]
		locations['Corner 3'] = [-1.2194, -3.1682, -0.9807, 0.1954]
		# charging_port = [-1.6756, -4.4751, -0.0184 0.9998]

		# Start patrolling flow 
		self.goal = MoveBaseGoal()
		rospy.loginfo("Start Patrolling")
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()
		rospy.sleep(2)


		for i in locations:
			coordinate = locations[i]
			destination = Pose(Point(coordinate[0], coordinate[1], 0.000), Quaternion(0.0, 0.0, coordinate[2], coordinate[3]))

			# Robot will go to point 1 by 1
			rospy.loginfo("Going to " + i)
			rospy.sleep(2)
			self.goal.target_pose.pose = destination
			self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
				rospy.loginfo("Reached " + i)
				rospy.sleep(2)
				# write location name to speech module report generation
				f = open("/home/mustar/catkin_ws/src/smoking-detection-mobile-surveillance-robot/location.txt", "w")
				f.write(i)
				f.close()
				self.start_scanning.publish('start')

			# Once the point is done, move to next location
			rospy.wait_for_message('point_done', String)
			rospy.sleep(2)

		# After visiting each corner, robot will go back to starting point
		rospy.loginfo("Going back initial point")
		rospy.sleep(2)
		initial_point = [0.0393,0.0335,-0.9807, 0.1954]
		self.goal.target_pose.pose = Pose(Point(initial_point[0], initial_point[1], 0.000), Quaternion(0.0, 0.0, initial_point[2], initial_point[3]))
		self.move_base.send_goal(self.goal)
		end_point = self.move_base.wait_for_result(rospy.Duration(300))
		if end_point == 1:
			rospy.loginfo("Reached initial point")
			rospy.sleep(2)

	def update_initial_pose(self, initial_pose):
		global original
		self.initial_pose = initial_pose
		if original == 0:
			self.origin = self.initial_pose.pose.pose
			original = 1

	def cleanup(self):
		rospy.loginfo("Shutting down navigation	....")
		self.move_base.cancel_goal()

if __name__=="__main__":
	rospy.init_node('navi_point')
	try:
		officeTour()
	except:
		pass
