#!/usr/bin/env python

"""
ROS node to track objects using DEEP_SORT TRACKER and YOLOv3 detector (darknet_ros)
Takes detected bounding boxes from darknet_ros and uses them to calculated tracked bounding boxes
Tracked objects and their ID are published to the sort_track node
For this reason there is a little delay in the publishing of the image that I still didn't solve
"""
import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from deep_sort.detection import Detection
from deep_sort import nn_matching
from deep_sort.tracker import Tracker
from deep_sort import generate_detections as gdet
from deep_sort import preprocessing as prep
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
	return (camera_topic, detection_topic, tracker_topic)

def callback_det(data):
	global detections
	global scores
	detections = []
	scores = []
	for box in data.bounding_boxes:
		detections.append(np.array([box.xmin, box.ymin, box.xmax-box.xmin, box.ymax-box.ymin]))
		scores.append(float('%.2f' % box.probability))
	detections = np.array(detections)


def callback_image(data):

	#Display Image
	bridge = CvBridge()
	cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
	#Features and detections
	features = encoder(cv_rgb, detections)
	detections_new = [Detection(bbox, score, feature) for bbox,score, feature in
                        zip(detections,scores, features)]
	# Run non-maxima suppression.
	boxes = np.array([d.tlwh for d in detections_new])
	scores_new = np.array([d.confidence for d in detections_new])
	indices = prep.non_max_suppression(boxes, 1.0 , scores_new)
	detections_new = [detections_new[i] for i in indices]

        # Call the tracker
        tracker.predict()
        tracker.update(detections_new)

	#Detecting bounding boxes
	for det in detections_new:
		bbox = det.to_tlbr()
		cv2.rectangle(cv_rgb,(int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(100,255,50), 1)
		cv2.putText(cv_rgb , "person", (int(bbox[0]), int(bbox[1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100,255,50), lineType=cv2.LINE_AA)

	#Tracker bounding boxes
        for track in tracker.tracks:
		if not track.is_confirmed() or track.time_since_update > 1:
                	continue
        	bbox = track.to_tlbr()
		msg.data = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]), track.track_id]
        	cv2.rectangle(cv_rgb, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 1)
            	cv2.putText(cv_rgb, str(track.track_id),(int(bbox[2]), int(bbox[1])),0, 5e-3 * 200, (255,255,255),1)
	
	cv2.imshow("YOLO+SORT", cv_rgb)
	cv2.waitKey(3)
		

def main():
    global tracker
    global encoder
    global msg
    msg = IntList()
    max_cosine_distance = 0.2
    nn_budget = 100
    metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
    tracker = Tracker(metric)
    model_filename = "/home/ilyas/darknetros_sort/src/sort_track/src/deep_sort/model_data/mars-small128.pb" #Change it to your directory
    encoder = gdet.create_box_encoder(model_filename)
    #Initialize ROS node
    rospy.init_node('sort_tracker', anonymous=True)
    rate = rospy.Rate(10)
    # Get the parameters
    (camera_topic, detection_topic, tracker_topic) = get_parameters()
    #Subscribe to image topic
    image_sub = rospy.Subscriber(camera_topic,Image,callback_image)
    #Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
    sub_detection = rospy.Subscriber(detection_topic, BoundingBoxes , callback_det)
    while not rospy.is_shutdown():
	#Publish results of object tracking
    	pub_trackers = rospy.Publisher(tracker_topic, IntList, queue_size=10)
	print(msg)
	pub_trackers.publish(msg)
	rate.sleep()
	#rospy.spin()


if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
