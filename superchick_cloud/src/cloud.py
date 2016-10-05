#! /usr/bin/env python

'''  ______________________________________________________________________
	/*
	*\	This code is part of the superchick project. 
	*/
	*\ 	Essentially, we are using the geometry_msgs 
	*/ 	transformed twist to rotate the point clouds whenever the face moves
	*\ 
	*/	Author: Olalekan Ogunmolu
	*\	Date: Oct 2, 2016
	*/__________________________________________________________________________
'''

# globals

import rospy

from geometry_msgs.msg import TransformStamped
from vicon_bridge.msg import Markers
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, Vector3Stamped, Vector3

import std_msgs.msg
import PyKDL
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2

class points_to_cloud():

	def __init__(self):
		# define globals
		from geometry_msgs.msg import TransformStamped as transform		
		self.markers = rospy.Subscriber("/vicon/markers", Markers, self.callback, queue_size = 10)
		self.transform = rospy.Subscriber("/vicon/Superdude/root", TransformStamped, self.t_callback, queue_size = 10)
		self.pcl_pub = rospy.Publisher("/vicon_clouds", PointCloud2, queue_size=10)

	def callback(self, markers):
		# rospy.loginfo(rospy.get_caller_id() + "\nSuperdude markers: \n%s \n", markers)		
		fore_marker 	= markers.markers[0]
		left_marker 	= markers.markers[1]
		right_marker 	= markers.markers[2]
		chin_marker 	= markers.markers[3]

		self.geometry_to_cloud2(fore_marker, left_marker, right_marker, chin_marker)

	def t_callback(self, transformed):	
		# rospy.loginfo(rospy.get_caller_id() + "\nSuperdude transform_stamped: \n%s \n", transformed)	
		self.transformer = transformed

		# print(transform)

	def geometry_to_cloud2(self, fore, left, right, chin):

		# rospy.loginfo("==> initialting publisher")
		# print(fore)
		rospy.sleep(0.5)
		fore_cloud = [(fore.translation.x)/1000, (fore.translation.y)/1000, (fore.translation.z)/1000]
		left_cloud = [(left.translation.x)/1000, (left.translation.y)/1000, (left.translation.z)/1000]
		right_cloud = [(right.translation.x)/1000, (right.translation.y)/1000, (right.translation.z)/1000] 
		chin_cloud =  [(chin.translation.x)/1000, (chin.translation.y)/1000, (chin.translation.z)/1000] 

		#cloud_header
		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'cloud_map'
		"""
		fore_kdl = PyKDL.Frame(PyKDL.Rotation.Quaternion(fore.rotation.x, fore.rotation.y,
	                                                 fore.rotation.z, fore.rotation.w),
							PyKDL.Vector(fore.transform.translation.x, 
							             fore.transform.translation.y, 
							             fore.transform.translation.z))

		left_kdl = PyKDL.Frame(PyKDL.Rotation.Quaternion(left.rotation.x, left.rotation.y,
		                                             left.rotation.z, left.rotation.w),
							PyKDL.Vector(left.transform.translation.x, 
							             left.transform.translation.y, 
							             left.transform.translation.z))

		right_kdl = PyKDL.Frame(PyKDL.Rotation.Quaternion(right.rotation.x, right.rotation.y,
		                                             right.rotation.z, right.rotation.w),
							PyKDL.Vector(right.transform.translation.x, 
							             right.transform.translation.y, 
							             right.transform.translation.z))

		chin_kdl = PyKDL.Frame(PyKDL.Rotation.Quaternion(chin.rotation.x, chin.rotation.y,
		                                             chin.rotation.z, chin.rotation.w),
							PyKDL.Vector(chin.transform.translation.x, 
							             chin.transform.translation.y, 
							             chin.transform.translation.z))

		fore_cloud2 = do_transform_cloud(fore_cloud, fore_kdl)
		left_cloud2 = do_transform_cloud(left_cloud, left_kdl)
		rigth_cloud2 = do_transform_cloud(rigth_cloud, rigth_kdl)
		chin_cloud2 = do_transform_cloud(chin_cloud, chin_kdl)
		"""
		cloud = [fore_cloud, left_cloud, right_cloud, chin_cloud]
		# print(cloud)
		#create pcl2 clouds from points
		scaled_pcl = pcl2.create_cloud_xyz32(header, cloud)	
		transformed_cloud = do_transform_cloud(scaled_pcl, self.transformer)

		self.pcl_pub.publish(transformed_cloud)

def main():
	p2c = points_to_cloud()
	rospy.init_node('super_listener', anonymous=True)
	try:
		rospy.spin()		
	except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()

