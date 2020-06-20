#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import Sofa
import math
import sys, os
import time
import logging
import datetime
import numpy as np
from utils import *
from config import *

from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec

logger = logging.getLogger(__name__)

# generate sinusoid trajectory for head
t, x = gen_sinusoid(amp=.8, freq=2, phase=30, interval=[0.1, 1, 0.01])

# https://www.sofa-framework.org/community/forum/topic/get-the-position-value-from-a-mechanicalobject-point-in-python/
def moveRestPos(rest_pos, pose):
	str_out = ' '
	dx, dy, dz = pose
	for i in range(0,len(rest_pos)) :
		str_out= str_out + ' ' + str(rest_pos[i][0]+dx)
		str_out= str_out + ' ' + str(rest_pos[i][1]+dy)
		str_out= str_out + ' ' + str(rest_pos[i][2]+dz)
	return str_out

# def rotateRestPos(rest_pos,rx,centerPosY,centerPosZ):
# 	str_out = ' '
# 	for i in xrange(0,len(rest_pos)) :
# 		newRestPosY = (rest_pos[i][1] - centerPosY)*math.cos(rx) - (rest_pos[i][2] - centerPosZ)*math.sin(rx) +  centerPosY
# 		newRestPosZ = (rest_pos[i][1] - centerPosY)*math.sin(rx) + (rest_pos[i][2] - centerPosZ)*math.cos(rx) +  centerPosZ
# 		str_out= str_out + ' ' + str(rest_pos[i][0])
# 		str_out= str_out + ' ' + str(newRestPosY)
# 		str_out= str_out + ' ' + str(newRestPosZ)
# 	return str_out

def rotateRestPos(rest_pos,rx,centerPosY,centerPosZ):
	str_out = ' '
	for i in xrange(0,len(rest_pos)) :
		newRestPosX = (rest_pos[i][1] - centerPosX)*math.cos(rx) - (rest_pos[i][2] - centerPosZ)*math.sin(rx) +  centerPosX
		newRestPosY = (rest_pos[i][1] - centerPosY)*math.sin(rx) + (rest_pos[i][2] - centerPosY)*math.cos(rx) +  centerPosY
		str_out= str_out + ' ' + str(newRestPosX)
		str_out= str_out + ' ' + str(newRestPosY)
		str_out= str_out + ' ' + str(rest_pos[i][2])
	return str_out

class controller(Sofa.PythonScriptController):
	'''
		For examples, see:

		+ Keyboard Control:
			- https://github.com/lakehanne/sofa/blob/master/examples/Tutorials/StepByStep/Dentistry_Python/keyboardControl.py
		+ Parallel and SSH Launcher:
			- https://github.com/lakehanne/sofa/blob/master/tools/sofa-launcher/launcher.py
		+ OneParticle:
			- https://github.com/lakehanne/sofa/blob/master/tools/sofa-launcher/example.py

		+ Note: +y points up, +x points right, +z out of the screen
		+ Note: -y points down, -x points left, -z into the screen
	'''
	def initGraph(self, root):
		self.move_dist = move_dist #(0, .40, 0)
		self.growth_rate = growth_rate #.5  #was .05
		self.max_pressure = max_pressure #100 # was 15
		# controls if IABs should continue to be inflated in a open-loop setting
		self.is_inflated = True
		self.deltaTime = root.findData('dt').value
		# print('deltaTime ', self.deltaTime, type(self.deltaTime))

		self._fig = plt.figure()
		self._gs = gridspec.GridSpec(1,1) # rows cols
		self.traj_plotter = HeadTrajPlotter(self._fig, self._gs[0]) # subplot in gridspec

		self.root = root

		self.dome_head = root.getChild('DomeHead')
		# obtain associated dofs and cavity dofs
		self.dome_head_dofs 	= self.get_dome_dofs(self.dome_head)
		dome_head_pose = self.dome_head_dofs.findData('rest_position').value

		self.thresholds = thresholds
		self.first_iter = True

		# logger.debug('patient initial pose {}'.format(thresholds['patient_trans']))


		self.is_chart_updated = False
		# use this to track the x, y and z positions of the patient over time
		self._x, self._y, self._z = [], [], []
		self._roll, self._yaw, self._pitch = [], [], []

		# visualization
		display_chart(self.run_traj_plotter)
		# plt.ioff()
		# plt.show()

		self.max_vals = 0 # maximum positional values in the patient
		self.turn_off_animation = False
		#
		self.centerPosX = 0 # 70
		self.centerPosY = 0
		self.rotAngle = 0

	# domes' mechanical states
	def get_dome_dofs(self, node):
		'dof name shall be in the form patient or base_neck etc'
		dh_dofs = node.getObject('dh_dofs')  # dome head
		# dh_collis_dofs = node.getObject('dh_collis_dofs')
		# cavity
		cav_node = node.getChild('DomeCavity')
		cav_dofs = cav_node.getObject('dome_cav_dofs')
		pressure_constraint = cav_node.getObject('SurfacePressureConstraint')
		# pressure_constraint_collis = node.getChild('dome_cav_collis_dofs')
		# dome cover back
		cover_node = node.getChild('DomeCover')
		cover_dofs = cover_node.getObject('dome_cover_dofs')
		# cover collis node
		cover_collis_node = node.getChild('DomeCoverCollis')
		cover_collis_dofs = cover_collis_node.getObject('dome_cover_collis_dofs')

		return Bundle(dict(dh_dofs=dh_dofs,
							cav_dofs=cav_dofs,
							pressure_constraint=pressure_constraint, # cavity
							cover_dofs=cover_dofs,
							cover_collis_dofs=cover_collis_dofs))

	def get_euler_angles(self, curr_points, y_vector):
		"""
		Find axis of rotation
		The cross product of the desired patient axis of rotation and the y_vector gives the axis of rotation
		perpendicular to the plane defined by the patient in a supine position and the x vector
		# Ref: Goncalo's answer here: https://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/

		We use y-vector because the y axis points up
		"""
		rot_axis = np.cross(curr_points, y_vector)
		# normalize the rotation axis
		rot_axis /= np.linalg.norm(rot_axis)
		# GET THE DIRECTION COSINE BTW CURRENT ROT AXES AND Y VECTOR
		dir_cosine = rot_axis.dot(y_vector)
		rot_angle = -1.0*np.arccos(dir_cosine)
		# generate quaternions: http://docs.ros.org/jade/api/tf/html/c++/Quaternion_8h_source.html#l00062
		s = np.sin(rot_angle * 0.5)/np.linalg.norm(rot_axis) # pg 34 Murray
		quaternions = [s*rot_axis[0], s*rot_axis[1], s*rot_axis[2], np.cos(rot_angle*.5)]
		fick_angles = R.from_quat(quaternions).as_euler('zyx', degrees=True) # sfrom scipy transforms
		self.euler_angles = Bundle(dict(yaw=fick_angles[0], pitch=fick_angles[1], roll=fick_angles[2]))

	def bwdInitGraph(self,node):
		# find the position at the end of the shape (which has the biggest x coordinate)
		Positions = self.patient_dofs.position
		max_x, max_y, max_z = 0, 0, 0
		max_idx_x, max_idx_y, max_idx_z = 0, 0, 0

		for i in range(len(Positions)):
			if Positions[i][0] > max_x:
				max_idx_x = i
				max_x = Positions[i][0]
			if Positions[i][1] > max_y:
				max_idx_y = i
				max_y = Positions[i][1]
			if Positions[i][2] > max_z:
				max_idx_z = i
				max_z = Positions[i][2]
		# calculate rotations given two points with respect to the origin
		# See https://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/
		rot_axis = [0.0, 1.0, 0.0] # since w
		self.get_euler_angles([max_x, max_y, max_z], rot_axis)

		max_ids = Bundle(dict(max_idx_x=max_idx_x, max_idx_y=max_idx_y, max_idx_z=max_idx_z, position=Positions))
		self.max_vals = Bundle(dict(max_x=max_x, max_y=max_y, max_z=max_z))
		logger.info('[x,y,z, yaw, pitch, roll {}, {}, {}, {}, {}, {}'.format(\
				max_x, max_y, max_z, self.euler_angles.yaw, self.euler_angles.pitch, self.euler_angles.roll))
		return 0

	def run_traj_plotter(self):
		if self.is_chart_updated:
			self.traj_plotter.update(self.data)
			# time.sleep(.11)
		self.is_chart_updated = False

	def update_head_pose(self):
		rest_pose = self.patient_dofs.findData('rest_position').value
		# rest pose is a lisrt
		x, y, z = [t[0] for t in rest_pose], [t[1] for t in rest_pose], [t[2] for t in rest_pose]
		# use 2-norm of x, y, and z
		self.data = np.linalg.norm(np.c_[x, y, z], axis=0)
		self.is_chart_updated = True

	def onBeginAnimationStep(self, deltaTime):
		self.deltaTime  += deltaTime

		# repopulate each iab at each time step
		self.base_neck_left		= 	self.root.getChild('base_neck_left')
		self.base_neck_right	= 	self.root.getChild('base_neck_right')
		self.base_skull_left	= 	self.root.getChild('base_skull_left')
		self.base_skull_right	= 	self.root.getChild('base_skull_right')
		# get side IABs
		self.side_fore_left		= 	self.root.getChild('side_fore_left')
		self.side_chin_left		= 	self.root.getChild('side_chin_left')
		self.side_fore_right	= 	self.root.getChild('side_fore_right')
		self.side_chin_right	= 	self.root.getChild('side_chin_right')
		# obtain associated dofs and cavity dofs
		self.base_neck_left_dofs 	= self.get_dome_dofs(self.base_neck_left)
		self.base_neck_right_dofs 	= self.get_dome_dofs(self.base_neck_right)
		self.base_skull_left_dofs 	= self.get_dome_dofs(self.base_skull_left)
		self.base_skull_right_dofs 	= self.get_dome_dofs(self.base_skull_right)

		# self.patient = self.root.getChild('patient')
		self.patient_dofs = self.patient.getObject('patient_dofs')

		if self.first_iter:
			rest_pat_pose = np.array([self.max_vals.max_x, self.max_vals.max_y, self.max_vals.max_z])
			# set targets
			self.thresholds['patient_trans'] = rest_pat_pose
			for i in range(3): self.thresholds['patient_trans'][i] += 100
			self.thresholds.update(self.thresholds)
			logger.debug('rest_pat_pose: {}, '.format(rest_pat_pose))
			self.first_iter = False

		curr_pat_pose = np.array([self.max_vals.max_x, self.max_vals.max_y, self.max_vals.max_z])
		"""
		# raise all the way to +x
		if curr_pat_pose[0]<self.thresholds['patient_trans'][0]: # not up to desired z
			pose = (self.growth_rate, 0, 0)
			test1 = moveRestPos(self.patient_dofs.findData('rest_position').value, pose)
			self.patient_dofs.findData('rest_position').value = test1
			self.patient_dofs.position = test1
			self._x.append(self.max_vals.max_x)
			if 	curr_pat_pose[0]>=self.thresholds['patient_trans'][0]-2:
				logger.info('finished inflating along x')
				stab_val= self._x[-1]
				for i in range(len(self._x)*4):
					self._x.append(stab_val)
		# raise all the way to +y
		if  (curr_pat_pose[1]<self.thresholds['patient_trans'][1]) and \
			(curr_pat_pose[0]>=self.thresholds['patient_trans'][0]): # not up to desired z
			pose = (0, self.growth_rate, 0)
			test2 = moveRestPos(self.patient_dofs.findData('rest_position').value, pose)
			self.patient_dofs.findData('rest_position').value = test2
			self.patient_dofs.position = test2
			self._y.append(self.max_vals.max_y)
			if 	curr_pat_pose[1]>=self.thresholds['patient_trans'][1]-2:
				logger.info('finished inflating along y')
				stab_val= self._y[-1]
				for i in range(len(self._y)*4):
					self._y.append(stab_val)
		# raise all the way to +z
		if  (curr_pat_pose[2]<self.thresholds['patient_trans'][2]) and \
			(curr_pat_pose[0]>=self.thresholds['patient_trans'][0]) and \
			(curr_pat_pose[1]>=self.thresholds['patient_trans'][1]): # not up to desired z
			pose = (0, 0, self.growth_rate)
			test3 = moveRestPos(self.patient_dofs.findData('rest_position').value, pose)
			self.patient_dofs.findData('rest_position').value = test3
			self.patient_dofs.position = test3
			self._z.append(self.max_vals.max_z)
			if 	curr_pat_pose[2]>=self.thresholds['patient_trans'][2]-2:
				logger.info('finished inflating along z')
				stab_val= self._z[-1]
				for i in range(len(self._z)*4):
					self._z.append(stab_val)
				lenx = len(self._x)
				self.record_all_data(np.c_[self._x[:lenx], self._y[:lenx], self._z[:lenx]])
				print('Finished animations. Recording all data.')
				self.root.getRootContext().animate = False
		"""
		if self.euler_angles.yaw<self.thresholds['patient_rot'][0]: # not up to desired z
			# rotate left about y
			dx = self.growth_rate*math.cos(self.rotAngle)
			dy = self.growth_rate*math.sin(self.rotAngle)
			pose = (dx, dy, 0.0)
			test = moveRestPos(self.patient_dofs.findData('rest_position').value, pose)
			self.patient_dofs.position = test
			self.centerPosX += dx
			self.centerPosY += dy
			self._yaw.append(self.euler_angles.yaw)
			if 	self.euler_angles.yaw>=self.thresholds['patient_rot'][0]-2:
				logger.info('finished inflating along yaw')
				stab_val= self._yaw[-1]
				for i in range(len(self._yaw)*4):
					self._yaw.append(stab_val)
		if self.euler_angles.pitch<self.thresholds['patient_rot'][1] and \
			(self.euler_angles.yaw>=self.thresholds['patient_rot'][0]): # not up to desired z
			# rotate left about y
			dx = self.growth_rate*math.cos(self.rotAngle)
			dy = self.growth_rate*math.sin(self.rotAngle)
			pose = (0.0, dx, dy)
			test2 = moveRestPos(self.patient_dofs.findData('rest_position').value, pose)
			self.patient_dofs.position = test2
			self.centerPosX += dx
			self.centerPosY += dy
			self._pitch.append(self.euler_angles.pitch)
			if 	self.euler_angles.pitch>=self.thresholds['patient_rot'][1]-2:
				logger.info('finished inflating along pitch')
				stab_val= self._pitch[-1]
				for i in range(len(self._pitch)*4):
					self._pitch.append(stab_val)
		if self.euler_angles.roll<self.thresholds['patient_rot'][2] and \
			(self.euler_angles.yaw>=self.thresholds['patient_rot'][0]) and \
			(self.euler_angles.pitch>=self.thresholds['patient_rot'][1]): # not up to desired z
			# rotate left about y
			dx = self.growth_rate*math.cos(self.rotAngle)
			dy = self.growth_rate*math.sin(self.rotAngle)
			pose = (dx, 0.0, dy)
			test2 = moveRestPos(self.patient_dofs.findData('rest_position').value, pose)
			self.patient_dofs.position = test2
			self.centerPosX += dx
			self.centerPosY += dy
			self._roll.append(self.euler_angles.roll)
			if 	self.euler_angles.roll>=self.thresholds['patient_rot'][2]-2:
				logger.info('finished inflating along roll')
				stab_val= self._roll[-1]
				for i in range(len(self._roll)*4):
					self._roll.append(stab_val)
				lenyaw = len(self._yaw)
				# self.record_all_data(np.c_[self._x[:lenx], self._y[:lenx], self._z[:lenx]])
				self.record_all_data(np.c_[self._yaw[:lenyaw], self._pitch[:lenyaw], self._roll[:lenyaw]])
				print('Finished animations. Recording all data.')
				self.root.getRootContext().animate = False

		return 0;

	def record_all_data(self, data):
		with open(self._pat_dofs_filename, 'a') as foo:
			arr_to_save = np.array(data)
			np.savetxt(foo, arr_to_save, delimiter=' ', fmt='%1.4e')
		self.root.getRootContext().animate = False
		logger.info('Stopped recording saved data points')

	def onEndAnimationStep(self, deltaTime):
		sys.stdout.flush()
		#access the 'position' state vector
		pat_poses = self.patient_dofs.findData('position').value
		self.bwdInitGraph(self.root)
		return 0;

	def onLoaded(self, node):
		return 0;

	def reset(self):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
		# usage e.g.
		if isPressed :
		   print("Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY))
		return 0;


	def onScriptEvent(self, senderNode, eventName,data):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def onMouseButtonRight(self, mouseX,mouseY,isPressed):
		## usage e.g.
		if isPressed :
		   print("Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY))
		return 0;

	def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
		## usage e.g.
		if isPressed :
		   print("Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY))
		return 0;
