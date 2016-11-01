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
import roslib
roslib.load_manifest('superchick_cloud')
import math
import tf

from geometry_msgs.msg import TransformStamped
from vicon_bridge.msg import Markers
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud 
from geometry_msgs.msg import Pose as pose
import geometry_msgs.msg
import PyKDL
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2



class points_to_cloud():

	def __init__(self):
		# define globals	
		self.markers = rospy.Subscriber("/vicon/delayed_markers", Markers, self.callback, queue_size = 10)
		self.transform = rospy.Subscriber("/vicon/head_transform", TransformStamped, self.t_callback, queue_size = 10)		
		self.pcl_pub = rospy.Publisher("/vicon/clouds", PointCloud2, queue_size=10)
		self.supername = rospy.get_param('/vicon/tf_ref_frame_id')

	def callback(self, markers):		
		fore_marker 	= markers.markers[0]
		left_marker 	= markers.markers[1]
		right_marker 	= markers.markers[2]
		chin_marker 	= markers.markers[3]

		self.geometry_to_cloud2(fore_marker, left_marker, right_marker, chin_marker)

	def t_callback(self, transform):	
		self.transformer = transform

	def handle_head_pose(self, supername):
		# this creates the transform from vicon to the head
	    br = tf.TransformBroadcaster()
	    yaw = 0#math.pi
	    pitch = 0
	    roll = 0
	    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	    br.sendTransform((-0.30, 0.2, 0.2),
	                     (q[0], q[1], q[2],q[3]),
	                     rospy.Time.now(),
	                     supername,
	                     "/base_link")

	    # self.transform_listener(self.supername)

	def transform_listener(self, supername):
		listener = tf.TransformListener()
		listener.lookupTransform(supername, "/base_link", rospy.Time(0))

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

		#create pcl2 clouds from points
		scaled_pcl = pcl2.create_cloud_xyz32(header, cloud)	
		transformed_cloud = scaled_pcl #do_transform_cloud(scaled_pcl, self.transformer)
		self.handle_head_pose(self.supername)
		self.pcl_pub.publish(transformed_cloud)

def main():
	rospy.init_node('super_listener', anonymous=True)	
	while not rospy.is_shutdown():		
		try:			
			p2c = points_to_cloud()
			r = rospy.Rate(100)  #publish at 100 Hz
			r.sleep()		    
			rospy.spin()	
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    continue	

if __name__ == '__main__':
    main()

