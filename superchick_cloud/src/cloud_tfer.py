#! /usr/bin/env python

'''  ______________________________________________________________________
	/*
	*\	This code is part of the superchick project. 
	*/
	*\ 	Essentially, we are publishing the transform from vicon to the 
	*/ 	head we created in moveit
	*\  
	*/  See http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
	*\   or http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
	*/
	*/	Author: Olalekan Ogunmolu
	*\	Date: Oct 27, 2016
	*/__________________________________________________________________________
'''

# globals
#This is a hack to position the point clouds directly on the move it stl file in rviz
import roslib
roslib.load_manifest('superchick_cloud')
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('super_tfer')

    listener = tf.Transformer(True, rospy.Duration(10.0))
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = '/world'
    m.child_frame_id = '/vicon/Superdude/head'
    m.transform.translation.x = 4.0
    m.transform.translation.y = 4
    m.transform.translation.z = 2
    m.transform.rotation.w = 1.0
    listener.setTransform(m)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/vicon/Superdude/head', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

        #"${0} ${0} ${-0.05}" rpy="${M_PI/2} 0 ${-M_PI}"
        #"0.3 ${bladder_pos_y+0.35} ${0.5}" rpy="${-M_PI/2} 0 0"