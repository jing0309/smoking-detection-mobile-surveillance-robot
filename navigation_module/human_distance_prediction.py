#!/usr/bin/env python

'''
	This python file allow you to just run distance prediction without waiting message from other module's publisher
'''

# import the necessary packages
import cv2
import math

class distancePrediction:
	def __init__(self):
		# variables
		# width of face in the real world or Object Plane
		self.known_width = 16 

		# Colors
		self.green = (0, 255, 0)
		self.red = (0, 0, 255)
		self.white = (255, 255, 255)
		
		# face detector parameter 
		# May change the parameter file address to detect fullbody or upperbody 
		self.face_detector = cv2.CascadeClassifier("/home/mustar/catkin_ws/src/jinghui_fyp_project/navigation_module/face_detect_and_distance_prediction/haarcascade_frontalface_default.xml")
		
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

		# read the image that detect smoker 
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

	# detect human face and return distance
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
			
		cv2.imshow("image", human_image)
		cv2.waitKey(0)

# -------------------------------------------------------------------------------------------
if __name__=="__main__":
	try:
		distancePrediction()
	except:
		pass
