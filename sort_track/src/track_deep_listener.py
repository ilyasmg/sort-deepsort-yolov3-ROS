#!/usr/bin/env python

#Listen to message published by deep_sort tracker (Bounding boxe of tracked object and ID)

import rospy
from sort_track.msg import IntList

def callback(data):
	print(data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('sort_track_deep', IntList , callback )

    rospy.spin()

if __name__ == '__main__':
    listener()
