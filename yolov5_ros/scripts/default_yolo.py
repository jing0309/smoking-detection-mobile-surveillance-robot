#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
                             

class Yolo_Dect:
    def __init__(self):
        waiting = rospy.wait_for_message('start_scanning', String)
        rospy.loginfo("Start person detection")
        # load parameters
        yolov5_path = rospy.get_param('/yolov5_path', '')
        image_topic = rospy.get_param(
            '~image_topic', '/usb_cam/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = rospy.get_param('~conf', '0.5')
        # load local repository(YoloV5:v6.0)
        self.model = torch.hub.load(yolov5_path, 'yolov5s', source='local')
        self.model.classes = [0]

        # which device will be used
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()

        self.model.conf = conf
        self.color_image = Image()
        self.depth_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)
        self.smoking_sub = rospy.Subscriber("smoking", String, self.repeat_process)
        # output publishers
        self.position_pub = rospy.Publisher(pub_topic,  BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher('/yolov5/detection_image', Image, queue_size=1)
        
        self.person_pub = rospy.Publisher('person', String, queue_size=10)

        # if no image messages
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def repeat_process(self, data):
        if data.data == "not smoking":
            main()
    
    def image_callback(self, image):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model(self.color_image)
        # xmin    ymin    xmax   ymax  confidence  class    name

        boxs = results.pandas().xyxy[0].values
        self.dectshow(self.color_image, boxs, image.height, image.width)
        
        cv2.waitKey(5000)
        if len(boxs) > 0:
            result = "Person detected"
            cv2.imwrite('/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/media/person_detected.jpg', image)
            f = open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/person_detected.txt", "w")
            f.write(result)
            f.close()
            self.person_pub.publish(result)
            rospy.sleep(5)
        else:
            result = "No person detected"
            self.person_pub.publish(result)

        with open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/person_detected.txt",'w') as file:
            pass

    def dectshow(self, org_img, boxs, height, width):
        img = org_img.copy()

        count = 0
        for i in boxs:
            count += 1

        for box in boxs:
            boundingBox = BoundingBox()
            boundingBox.probability =np.float64(box[4])
            # added
            confidence = boundingBox.probability
            boundingBox.xmin = np.int64(box[0])
            boundingBox.ymin = np.int64(box[1])
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])
            boundingBox.num = np.int16(count)
            boundingBox.Class = box[-1]
            # added
            label = boundingBox.Class

            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color

            cv2.rectangle(img, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)

            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10
            
            # added
            text = ('{:s}: {:.3f}').format(label, confidence)
            cv2.putText(img, text,
                        (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)


            self.boundingBoxes.bounding_boxes.append(boundingBox)
            self.position_pub.publish(self.boundingBoxes)

        self.publish_image(img, height, width)
        cv2.imshow('YOLOv5 Person Detection', img)
        

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('yolov5_person', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()

if __name__ == "__main__":
    main()