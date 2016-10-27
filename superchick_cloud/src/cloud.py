#! /usr/bin/env python

'''  ______________________________________________________________________
	/*
	*\	This code is part of the superchick project. 
	*/
	*\ 	Essentially, we are using the geometry_msgs 
	*/ 	transformed twist to rotate the point clouds whenever the face moves
	*\  
	*/  Set fixed frame to the name of your object in vicon 
	*\  e.g. /vicon/<segment name>/<subject name>
	*/
	*/	Author: Olalekan Ogunmolu
	*\	Date: Oct 2, 2016
	*/__________________________________________________________________________
'''

# globals

import rospy

from geometry_msgs.msg import TransformStamped
from vicon_bridge.msg import Markers
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud 

import std_msgs.msg
import PyKDL
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2

class points_to_cloud():

	def __init__(self):
		# define globals	
		self.markers = rospy.Subscriber("/vicon/markers", Markers, self.callback, queue_size = 10)
		self.transform = rospy.Subscriber("/vicon/Superdude/head", TransformStamped, self.t_callback, queue_size = 10)		
		self.pcl_pub = rospy.Publisher("/vicon_clouds", PointCloud2, queue_size=10)

	def callback(self, markers):
		# rospy.loginfo(rospy.get_caller_id() + "\nSuperdude markers: \n%s \n", markers)		
		fore_marker 	= markers.markers[0]
		left_marker 	= markers.markers[1]
		right_marker 	= markers.markers[2]
		chin_marker 	= markers.markers[3]

		self.geometry_to_cloud2(fore_marker, left_marker, right_marker, chin_marker)

	def t_callback(self, transform):	
		# rospy.loginfo(rospy.get_caller_id() + "\nSuperdude transform_stamped: \n%s \n", transformed)	
		self.transformer = transform

	def geometry_to_cloud2(self, fore, left, right, chin):

		fore_cloud = [(fore.translation.x)/1000, (fore.translation.y)/1000, (fore.translation.z)/1000]
		left_cloud = [(left.translation.x)/1000, (left.translation.y)/1000, (left.translation.z)/1000]
		right_cloud = [(right.translation.x)/1000, (right.translation.y)/1000, (right.translation.z)/1000] 
		chin_cloud =  [(chin.translation.x)/1000, (chin.translation.y)/1000, (chin.translation.z)/1000] 

		#cloud_header
		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'cloud_map'
		cloud = [fore_cloud, left_cloud, right_cloud, chin_cloud]
		# print(cloud)
		#create pcl2 clouds from points
		scaled_pcl = pcl2.create_cloud_xyz32(header, cloud)	
		# print(self.transformer)
		transformed_cloud = do_transform_cloud(scaled_pcl, self.transformer)

		self.pcl_pub.publish(transformed_cloud)

def main():
	rospy.init_node('super_listener', anonymous=True)	
	while not rospy.is_shutdown():		
		try:			
			p2c = points_to_cloud()
			r = rospy.Rate(30)  #publish at 100 Hz
			r.sleep()
			rospy.spin()		
		except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()

