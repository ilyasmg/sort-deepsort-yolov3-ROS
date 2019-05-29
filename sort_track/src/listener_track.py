#!/usr/bin/env python

#Listen to message published by sort tracker (Bounding boxe of tracked object and ID)
_
import rospy
from sort_track.msg import IntList

def callback(data):
	print(data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('sort_track', IntList , callback )

    rospy.spin()

if __name__ == '__main__':
    listener()
