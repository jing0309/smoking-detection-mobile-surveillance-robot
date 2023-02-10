#!/usr/bin/env python
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
        waiting = rospy.wait_for_message('distance_prediction', String)
        rospy.loginfo("Start to detect smoker...")
        # load parameters
        self.detection_done = rospy.Publisher('smoke_detect_done', String, queue_size=10)
        yolov5_path = rospy.get_param('/yolov5_path', '')
        weight_path = rospy.get_param('~weight_path', '')
        image_path = rospy.get_param('~image_path', '/home/mustar/Documents/hand.jpeg')
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        conf = rospy.get_param('~conf', '0.5')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        # load local repository(YoloV5:v6.0)
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')

        self.boundingBoxes = BoundingBoxes()
        self.position_pub = rospy.Publisher(pub_topic,  BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolov5/detection_image',  Image, queue_size=1)

        # which device will be used
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()

        # Load class color
        self.classes_colors = {}
        self.model.conf = conf
        self.color_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        # self.color_image = cv2.cvtColor(self.color_image, cv2.CCOLOR_BGR2RGB)
        height = self.color_image.shape[0]
        width = self.color_image.shape[1]

        results = self.model(self.color_image)
        boxs = results.pandas().xyxy[0].values
        self.dectshow(self.color_image, boxs, height, width)

        # cv2.waitKey(0)

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

            x_min = int(box[0])
            y_min = int(box[1])
            x_max = int(box[2])
            y_max = int(box[3])

            cv2.rectangle(img, (x_min, y_min),
                          (x_max, y_max), (int(color[0]),int(color[1]), int(color[2])), 2)

            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10
            
            # added
            text = ('{:s}: {:.3f}').format(label, confidence)
            cv2.putText(img, text,
                        (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            bbox_list = [x_min, y_min, x_max, y_max]
            f = open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/smoking_result_yolo.txt", "w")
            for elem in range(len(bbox_list)):
                # print(elem)
                f.write(str(bbox_list[elem]) + '\n')
            f.close()

            self.boundingBoxes.bounding_boxes.append(boundingBox)
            self.position_pub.publish(self.boundingBoxes)
            
        self.publish_image(img, height, width)
        cv2.imshow('YOLOv5 Smoking Detection', img)
        cv2.imwrite('/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/media/smoker.jpg', img)
        self.detection_done.publish("done")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # with open("/home/mustar/catkin_ws/src/Yolov5_ros/yolov5_ros/yolov5_ros/smoking_result_yolo.txt",'w') as file:
        #     pass

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
    rospy.init_node('yolov5_ros', anonymous=True)
    while not rospy.is_shutdown():
        yolo_dect = Yolo_Dect()


if __name__ == "__main__":
    main()