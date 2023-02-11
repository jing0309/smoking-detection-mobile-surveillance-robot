#!/usr/bin/env python

# import the necessary packages
import cv2
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import math

class distancePrediction:
	def __init__(self):
		while not rospy.is_shutdown():
			rospy.loginfo("start distance prediction")

			# ================== INITIALIZATION ==================
			# Publisher that notify the distance prediction process is done 
			self.complete_action = rospy.Publisher('distance_prediction', String, queue_size=10)
			
			# variables
			# width of face in the real world or Object Plane
			self.known_width = 16

			# Colors
			self.green = (0, 255, 0)
			self.red = (0, 0, 255)
			self.white = (255, 255, 255)
			
			# face detector parameter 
			# May change the parameter file address to detect fullbody or upperbody 
			self.face_detector = cv2.CascadeClassifier("/home/mustar/catkin_ws/src/smoking-detection-mobile-surveillance-robot/navigation_module/parameter/haarcascade_frontalface_default.xml")

			# focal length of the camera
			self.focal_length_found = 1009.66
			
			'''
				# If you would like to recalculate the focal length using own photo and values 
				# reading reference image from directory
				ref_image = cv2.imread("/home/mustar/catkin_ws/src/smoking-detection-mobile-surveillance-robot/navigation_module/image/your_image_name.jpg")
				
				# distance from camera to object(face) measured
				self.known_distance = 60 # in centimeter
				
				# reading the face width from reference image
				self.ref_image_face_width, centre_y = self.face_data(ref_image)
				self.focal_length_found = self.focal_length(self.known_distance, self.known_width, self.ref_image_face_width)
				cv2.imshow("ref_image", ref_image)
			'''

			# Publisher For the robot to move towards smoker 
			self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
			# ====================================================

			# Waiting for message to start distance prediction
			while True:
				self.start_indicator = rospy.wait_for_message("start_distance_prediction", String)
				if self.start_indicator.data == "done":
					break
			
			# Start distance prediction
			rospy.loginfo("Start distance prediction")
			self.distance_prediction("/home/mustar/catkin_ws/src/smoking-detection-mobile-surveillance-robot/yolov5_ros/media/person_detected.jpg")

	# focal length finder function
	def focal_length(self, measured_distance, real_width, width_in_rf_image):
		# Calculate the Focal Length of robot camera
		focal_length_value = (width_in_rf_image * measured_distance) / real_width
		return focal_length_value

	# distance estimation function
	def distance_finder(self, focal_length, real_face_width, face_width_in_frame):
		# Estimates the distance between object and camera using arguments(focal_length, Actual_object_width, Object_width_in_the_image)
		distance = (real_face_width * focal_length) / face_width_in_frame
		return distance

	# face detector function
	def face_data(self, image):
		# Detect the face by taking image as argument and returns face_width in the pixels
		face_width = 0
		cy = 0  # coordinate of center of bounding box at y-axis
		gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# Detect face(s)
		faces = self.face_detector.detectMultiScale(gray_image, 1.3, 5)
		for (x, y, h, w) in faces:
			# draw bounding boxes at face detected
			cv2.rectangle(image, (x, y), (x + w, y + h), self.white, 1)
			# image face width = width of bounding box 
			face_width = w
			# cy is calculated by divide the height of bounding box by 2
			cy = int(y + h / 2)
		return face_width, cy

	def distance_prediction(self, image_path):
		# Read the person detected image
		human_image = cv2.imread(image_path)
		# center of image for y
		y_center = 480 / 2

		# calling face_data function
		face_width_in_frame, cy = self.face_data(human_image)
		# If face is detected
		if face_width_in_frame != 0:
			# Calculate the distance by using the formula 
			distance = self.distance_finder(self.focal_length_found, self.known_width, face_width_in_frame)

			height_from_face_to_centre = abs(cy - y_center)
			# Calculate the actual distance by using pythagoras theorem
			# for detailed, please refer to the report
			dist = math.sqrt(abs(distance**2 - height_from_face_to_centre**2))
			text = 'Distance = ' + str(round(dist,2)) + ' CM'

			# Drwaing Text on the screen
			cv2.putText(
				human_image, text, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (self.red), 2
			)

			# create one Twist() variables to moving forward
			# let's go forward at 0.2 m/s
			move_cmd = Twist()
			move_cmd.linear.x = 0.2
			# calculate the number of messages to publish to move_cmd to move towards 1.5 meter in front of target
			num_publish_message = int(distance / (0.2 * 10) - 150 / (0.2 * 10))
			if num_publish_message > 0:
				for x in range(0, num_publish_message):
					self.cmd_vel.publish(move_cmd)
					rospy.sleep(0.1)
		
		# Take photo for smoking detection module
		os.system('rosrun navigation_module take_photo.py')
		# Publish messgae to notify that distance prediction is done
		self.complete_action.publish('done')
		rospy.loginfo("end distance prediction")
		# cv2.imshow('image', human_image)
		# cv2.waitKey(0)


# -------------------------------------------------------------------------------------------
if __name__=="__main__":
	rospy.init_node('distance_prediction')
	try:
		distancePrediction()
	except:
		pass
