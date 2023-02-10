#!/usr/bin/env python3
# Import Libraries
import cv2
import mediapipe as mp
import time
import numpy as np
import rospy
from std_msgs.msg import String

def draw_bounding_box(image, landmarks, label):
    keypoint_pos = np.empty((0, 2), int)
    height, width, _ = image.shape

    for i in landmarks:
        # Acquire x, y but don't forget to convert to integer.
        x = min(int(i.x * width), width-1)
        y = min(int(i.y * height), height-1)
        landmark_point = [np.array((x, y))]
        keypoint_pos = np.append(keypoint_pos, landmark_point, axis=0)
    x, y, w, h = cv2.boundingRect(keypoint_pos)
    bbox = [x, y, x + w, y + h]
    x_min = bbox[0]-20
    y_min = bbox[1]-20
    x_max = bbox[2]
    y_max = bbox[3]
    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 0, 0), 2)
    cv2.putText(image, label, (bbox[0], bbox[1]-25), cv2.FONT_HERSHEY_COMPLEX, 0.7, (20,255,155), 2)
    bbox_list = [x_min, y_min, x_max, y_max]
    f = open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/smoking_result_mp.txt", "w")
    f.write(label+'\n')
    for elem in range(len(bbox_list)):
        f.write(str(bbox_list[elem]) + '\n')
    f.close()

def main():
    waiting = rospy.wait_for_message('smoke_detect_done', String)
    detection_done = rospy.Publisher('face_detect_done', String, queue_size=10)
    rospy.loginfo("Start to detect face and hand...")
    image_path = rospy.get_param('~image_path', '/home/mustar/Documents/hand.jpeg')
    
    # Grabbing the Holistic Model from Mediapipe and
    # Initializing the Model
    mp_holistic = mp.solutions.holistic
    holistic_model = mp_holistic.Holistic(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    img = cv2.imread(image_path)

    imgCopy = img.copy()

    results = holistic_model.process(imgCopy)

    if results.face_landmarks:
        face_landmark = results.face_landmarks.landmark
        draw_bounding_box(imgCopy, face_landmark, "Face")

    if results.left_hand_landmarks:
        left_hand_landmark = results.left_hand_landmarks.landmark
        draw_bounding_box(imgCopy, left_hand_landmark, "L_Hand")
            
    if results.right_hand_landmarks:
        right_hand_landmark = results.right_hand_landmarks.landmark
        draw_bounding_box(imgCopy, right_hand_landmark, "R_Hand")

    # Display the resulting image
    cv2.imshow("Facial and Hand Landmarks", imgCopy)
    cv2.imwrite('/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/media/facehandLabeled.jpg', imgCopy)
    detection_done.publish("done")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('mp_node', anonymous=True)
    while not rospy.is_shutdown():
        main()