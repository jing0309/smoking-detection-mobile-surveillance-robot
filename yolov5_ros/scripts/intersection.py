#!/usr/bin/env python3
import matplotlib.path as mplPath
import rospy
from std_msgs.msg import String


def main():
    waiting = rospy.wait_for_message('face_detect_done', String)
    # Initialize scanning result publisher
    smoking_node = rospy.Publisher('smoking_topic', String, queue_size=10)
    point_done = rospy.Publisher('point_done', String, queue_size=10)
    rospy.loginfo("Start to check for intersection...")

    # Get coordinates of bounding boxes from YOLOv5 and MediaPipe
    f_mp =  open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/smoking_result_mp.txt","r")
    result_mp = f_mp.read()
    f_mp.close()
    bbox_mp = result_mp.split('\n')

    mp_dict = {}
    for i in range(0,len(bbox_mp)-1,5):
        mp_dict[bbox_mp[i]] = [bbox_mp[i+1], bbox_mp[i+2], bbox_mp[i+3], bbox_mp[i+4]]

    if mp_dict.get('Face') != None:
        coors = mp_dict['Face']
        xmin_mp = coors[0]
        ymin_mp = coors[1]
        xmax_mp = coors[2]
        ymax_mp = coors[3]

        face_mediapipe_bbox = mplPath.Path([(xmin_mp, ymin_mp), (xmin_mp, ymax_mp), (xmax_mp, ymax_mp), 
        (xmax_mp, ymin_mp)])
    
    if mp_dict.get('L_Hand') != None:
        coors = mp_dict['L_Hand']
        xmin_mp = coors[0]
        ymin_mp = coors[1]
        xmax_mp = coors[2]
        ymax_mp = coors[3]

        lhand_mediapipe_bbox = mplPath.Path([(xmin_mp, ymin_mp), (xmin_mp, ymax_mp), (xmax_mp, ymax_mp), 
        (xmax_mp, ymin_mp)])

    if mp_dict.get('R_Hand') != None:
        coors = mp_dict['R_Hand']
        xmin_mp = coors[0]
        ymin_mp = coors[1]
        xmax_mp = coors[2]
        ymax_mp = coors[3]

        rhand_mediapipe_bbox = mplPath.Path([(xmin_mp, ymin_mp), (xmin_mp, ymax_mp), (xmax_mp, ymax_mp), 
        (xmax_mp, ymin_mp)])

    f_yolo =  open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/smoking_result_yolo.txt","r")
    result_yolo = f_yolo.read()
    f_yolo.close()
    if result_yolo != "":
        bbox_yolo = result_yolo.split('\n')

        xmin_yolo = bbox_yolo[0]
        ymin_yolo = bbox_yolo[1]
        xmax_yolo = bbox_yolo[2]
        ymax_yolo = bbox_yolo[3]

        yolov5_bbox = mplPath.Path([(xmin_yolo, ymin_yolo), (xmin_yolo, ymax_yolo), 
        (xmax_yolo, ymax_yolo), (xmax_yolo, ymin_yolo)])
        
        # Check for intersection
        if mp_dict.get('Face') != None:
            if yolov5_bbox.intersects_path(face_mediapipe_bbox):
                rospy.sleep(5)
                smoking_node.publish("smoking")
                rospy.loginfo("Bounding boxes intersect! Smoking detected.")
        elif mp_dict.get('L_Hand') != None:
            if yolov5_bbox.intersects_path(lhand_mediapipe_bbox):
                rospy.sleep(5)
                smoking_node.publish("smoking")
                rospy.loginfo("Bounding boxes intersect! Smoking detected.")
        elif mp_dict.get('R_Hand') != None:
            if yolov5_bbox.intersects_path(rhand_mediapipe_bbox):
                rospy.sleep(5)
                smoking_node.publish("smoking")
                rospy.loginfo("Bounding boxes intersect! Smoking detected.")
        else:
            rospy.sleep(5)
            point_done.publish("done")
            rospy.loginfo("Bounding boxes do not intersect. No smoking detected.")
    else:
        rospy.sleep(5)
        point_done.publish("done")
        rospy.loginfo("Bounding boxes do not intersect. No smoking detected.")

    with open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/smoking_result_yolo.txt",'w') as file:
        pass

    with open("/home/mustar/catkin_ws/src/fyp_yc/Yolov5_ros/yolov5_ros/yolov5_ros/smoking_result_mp.txt",'w') as file:
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('intersect_node',anonymous=True)
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass