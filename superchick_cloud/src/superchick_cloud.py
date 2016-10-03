#! /usr/bin/env python

'''
	/*
	*  
	\	This code is part of the superchick project. 
	\ 	Essentially, we are transforming the geometry_msgs 
	\ 	transformed data to pointcloud2 messages.
	\
	\ 
	\	Author: Olalekan Ogunmolu
	\	Date: Oct 2, 2016
	*/
'''

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from vicon_bridge.msg import Markers
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud 
from geometry_msgs.msg import do_transform_point

def callback(markers):
	# rospy.loginfo(rospy.get_caller_id() + "\nSuperdude markers: \n%s \n", markers.markers)
	fore_marker 	= markers.markers[0]
	left_marker 	= markers.markers[1]
	right_marker 	= markers.markers[2]
	chin_marker 	= markers.markers[3]

	print("fore_marker: ", fore_marker)
	# print "\n"
	print("left_marker: ", left_marker)
	# print "\n"
	print("right_marker: " ,right_marker)
	# print "\n"
	print("chin_marker: ", chin_marker)
	# print "\n"

	geometry_to_cloud2(fore_marker, left_marker, right_marker, chin_marker)

def geometry_to_cloud2(fore, left, right, chin):
	fore_cloud 	= do_transform_point(fore, transform)
	left_cloud 	= do_transform_point(left, transform)
	right_cloud = do_transform_point(right, transform)
	chin_cloud 	= do_transform_point(chin, transform)




def markers_listener():
	rospy.init_node('super_listener', anonymous=True)

	markers = rospy.Subscriber("/vicon/markers", Markers, callback)

	rospy.spin()

if __name__ == '__main__':
	markers_listener()


