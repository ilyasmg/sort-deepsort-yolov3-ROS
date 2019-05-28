#!/usr/bin/env python

"""
ROS node to track objects using SORT TRACKER and YOLOv3 detector (darknet_ros)
Takes detected bounding boxes from darknet_ros and uses them to calculated tracked bounding boxes
Tracked objects and their ID are published to the sort_track node
No delay here
"""

import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from sort import sort 
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sort_track.msg import IntList

def get_parameters():
	"""
	Gets the necessary parameters from .yaml file
	Returns tuple
	"""
	camera_topic = rospy.get_param("~camera_topic")
	detection_topic = rospy.get_param("~detection_topic")
	tracker_topic = rospy.get_param('~tracker_topic')
	cost_threhold = rospy.get_param('~cost_threhold')
	min_hits = rospy.get_param('~min_hits')
	max_age = rospy.get_param('~max_age')

	return (camera_topic, detection_topic, tracker_topic, cost_threhold, max_age, min_hits)


def callback_det(data):
	global detections
	global trackers
	global track
	detections = []
	trackers = []
	track = []
	for box in data.bounding_boxes:
		detections.append(np.array([box.xmin, box.ymin, box.xmax, box.ymax, round(box.probability,2)]))
		
	detections = np.array(detections)
	#Call the tracker
	trackers = tracker.update(detections)
	trackers = np.array(trackers, dtype='int')
	track = trackers
	msg.data = track
def callback_image(data):

	#Display Image
	bridge = CvBridge()
	cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
	#TO DO: FIND BETTER AND MORE ACCURATE WAY TO SHOW BOUNDING BOXES!!
	#Detection bounding box
	cv2.rectangle(cv_rgb, (int(detections[0][0]), int(detections[0][1])), (int(detections[0][2]), int(detections[0][3])), (100, 255, 50), 1)
	cv2.putText(cv_rgb , "person", (int(detections[0][0]), int(detections[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 255, 50), lineType=cv2.LINE_AA)
	
	#Tracker bounding box
	cv2.rectangle(cv_rgb, (track[0][0], track[0][1]), (track[0][2], track[0][3]), (255, 255, 255), 1)
	cv2.putText(cv_rgb , str(track[0][4]), (track[0][2], track[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)
	
	cv2.imshow("YOLO+SORT", cv_rgb)
	cv2.waitKey(3)


def main():
    global tracker
    global msg
    msg = IntList()
    while not rospy.is_shutdown():
	#Initialize ROS node
        rospy.init_node('sort_tracker', anonymous=False)
	rate = rospy.Rate(10)
        # Get the parameters
        (camera_topic, detection_topic, tracker_topic, cost_threshold, max_age, min_hits) = get_parameters()
	tracker = sort.Sort(max_age=max_age, min_hits=min_hits) #create instance of the SORT tracker
	cost_threshold = cost_threshold
	#Subscribe to image topic
	image_sub = rospy.Subscriber(camera_topic,Image,callback_image)
        #Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
	sub_detection = rospy.Subscriber(detection_topic, BoundingBoxes , callback_det)
	#Publish results of object tracking
	pub_trackers = rospy.Publisher(tracker_topic, IntList, queue_size=10)
	#print(msg) #Testing msg that is published
	#pub_trackers.publish(msg)
	rate.sleep()
	rospy.spin()


if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
