#! /usr/bin/env python

'''
	/*
	*\	This code is part of the superchick project. 
	*\ 	Essentially, we are transforming the geometry_msgs 
	*\ 	transformed data to pointcloud2 messages.
	*\ 
	*\	Author: Olalekan Ogunmolu
	*\	Date: Oct 2, 2016
	*/
'''

# globals

import rospy

from geometry_msgs.msg import TransformStamped
from vicon_bridge.msg import Markers
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, Vector3Stamped, Vector3

import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2

def callback(markers):
	# rospy.loginfo(rospy.get_caller_id() + "\nSuperdude markers: \n%s \n", markers.markers)		
	fore_marker 	= markers.markers[0]
	left_marker 	= markers.markers[1]
	right_marker 	= markers.markers[2]
	chin_marker 	= markers.markers[3]

	geometry_to_cloud2(fore_marker, left_marker, right_marker, chin_marker)

def geometry_to_cloud2(fore, left, right, chin):

	pcl_pub = rospy.Publisher("/vicon_clouds", PointCloud2)
	rospy.loginfo("==> initialting publisher")
	rospy.sleep(0.5)
	clouds = [ 	
				[(fore.translation.x)/1000, (fore.translation.y)/1000, (fore.translation.z)/1000], 
				[(left.translation.x)/1000, (left.translation.y)/1000, (left.translation.z)/1000], 
				[(right.translation.x)/1000, (right.translation.y)/1000, (right.translation.z)/1000], 
				[(chin.translation.x)/1000, (chin.translation.y)/1000, (chin.translation.z)/1000] 
			]
	print "===> clouds\n"
	print(clouds)

	#cloud_header
	header = std_msgs.msg.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'cloud_map'

	#create pcl from points
	scaled_pcl = pcl2.create_cloud_xyz32(header, clouds)
	#publish    
	rospy.loginfo("happily publishing vicon pointcloud.. !")
	pcl_pub.publish(scaled_pcl)


def markers_listener():
	rospy.init_node('super_listener', anonymous=True)
	markers = rospy.Subscriber("/vicon/markers", Markers, callback)

	rospy.spin()

if __name__ == '__main__':
	markers_listener()


