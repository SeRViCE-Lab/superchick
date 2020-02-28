#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import Sofa
import math
import sys
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

def moveRestPos(rest_pos, dx, dy, dz):
	str_out = ' '
	for i in xrange(0,len(rest_pos)) :
		str_out= str_out + ' ' + str(rest_pos[i][0]+dx)
		str_out= str_out + ' ' + str(rest_pos[i][1]+dy)
		str_out= str_out + ' ' + str(rest_pos[i][2]+dz)
	return str_out

def rotateRestPos(rest_pos,rx,centerPosY,centerPosZ):
	str_out = ' '
	for i in xrange(0,len(rest_pos)) :
		newRestPosY = (rest_pos[i][1] - centerPosY)*math.cos(rx) - (rest_pos[i][2] - centerPosZ)*math.sin(rx) +  centerPosY
		newRestPosZ = (rest_pos[i][1] - centerPosY)*math.sin(rx) + (rest_pos[i][2] - centerPosZ)*math.cos(rx) +  centerPosZ
		str_out= str_out + ' ' + str(rest_pos[i][0])
		str_out= str_out + ' ' + str(newRestPosY)
		str_out= str_out + ' ' + str(newRestPosZ)
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

		self.patient = root.getChild('patient')
		self.patient_dofs = self.patient.getObject('patient_dofs')
		pat_rest_pose = self.patient_dofs.findData('rest_position').value

		self.thresholds = thresholds
		# thresholds['patient_trans'] = np.linalg.norm(pat_rest_pose, axis=0)
		# x, y, z = [t[0] for t in pat_rest_pose], [t[1] for t in pat_rest_pose], [t[2] for t in pat_rest_pose]
		# logger.debug('rest pose len x: {}, y: {}, z: {} rest_pose: {}: '.format(\
		# 		len(x), len(y), len(z), len(pat_rest_pose)))
		# print(self.patient_dofs)
		# thresholds.update(thresholds)
		# print('thresholds: ', thresholds)
		self.first_iter = True

		self.root = root
		logger.debug('patient initial pose {}'.format(thresholds['patient_trans']))

		# get base IABs
		self.base_neck_left		= 	root.getChild('base_neck_left')
		self.base_neck_right	= 	root.getChild('base_neck_right')
		self.base_skull_left	= 	root.getChild('base_skull_left')
		self.base_skull_right	= 	root.getChild('base_skull_right')
		# get side IABs
		self.side_fore_left		= 	root.getChild('side_fore_left')
		self.side_chin_left		= 	root.getChild('side_chin_left')
		self.side_fore_right	= 	root.getChild('side_fore_right')
		self.side_chin_right	= 	root.getChild('side_chin_right')
		# obtain associated dofs and cavity dofs
		self.base_neck_left_dofs 	= self.get_dome_dofs(self.base_neck_left)
		self.base_neck_right_dofs 	= self.get_dome_dofs(self.base_neck_right)
		self.base_skull_left_dofs 	= self.get_dome_dofs(self.base_skull_left)
		self.base_skull_right_dofs 	= self.get_dome_dofs(self.base_skull_right)

		self.is_chart_updated = False

		# visualization
		display_chart(self.run_traj_plotter)
		# plt.ioff()
		# plt.show()

	# domes' mechanical states
	def get_dome_dofs(self, node):
		'dof name shall be in the form patient or base_neck etc'
		dh_dofs = node.getObject('dh_dofs')  # dome head
		# dh_collis_dofs = node.getObject('dh_collis_dofs')
		# cavity
		cav_node = node.getChild('DomeCavity')
		pressure_constraint = cav_node.getObject('SurfacePressureConstraint')
		# pressure_constraint_collis = node.getChild('dome_cav_collis_dofs')
		# dome cover back
		cover_node = node.getChild('DomeCover')
		cover_dofs = cover_node.getObject('dome_cover_dofs')
		# cover collis node
		cover_collis_node = node.getChild('DomeCoverCollis')
		cover_collis_dofs = cover_collis_node.getObject('dome_cover_collis_dofs')

		return Bundle(dict(dh_dofs=dh_dofs,
						pressure_constraint=pressure_constraint, # cavity
						cover_dofs=cover_dofs,
						cover_collis_dofs=cover_collis_dofs))

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


	def onKeyPressed(self,c):
		self.dt = self.root.findData('dt').value
		incr = self.dt*1000.0;

		if (c == "+"):
			print(' raising head using base IABs')
			bnl_val = self.base_neck_left_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate
			bnr_val = self.base_neck_right_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate
			bsl_val = self.base_skull_left_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate
			bsr_val = self.base_skull_right_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate

			bnl_val = max_pressure if bnl_val > max_pressure else bnl_val
			bnr_val = max_pressure if bnr_val > max_pressure else bnr_val
			bsl_val = max_pressure if bsl_val > max_pressure else bsl_val
			bsr_val = max_pressure if bsr_val > max_pressure else bsr_val

			self.base_neck_left_dofs.pressure_constraint.findData('value').value = str(bnl_val)
			self.base_neck_right_dofs.pressure_constraint.findData('value').value = str(bnr_val)
			self.base_skull_left_dofs.pressure_constraint.findData('value').value = str(bsl_val)
			self.base_skull_right_dofs.pressure_constraint.findData('value').value = str(bsr_val)

			self.update_head_pose()
			plt.ion()
			plt.show()

		if (c == "-"):
			print('lowering head using base IABs')
			bnl_val = self.base_neck_left_dofs.pressure_constraint.findData('value').value[0][0] - self.growth_rate
			bnr_val = self.base_neck_right_dofs.pressure_constraint.findData('value').value[0][0] - self.growth_rate
			bsl_val = self.base_skull_left_dofs.pressure_constraint.findData('value').value[0][0] - self.growth_rate
			bsr_val = self.base_skull_right_dofs.pressure_constraint.findData('value').value[0][0] - self.growth_rate

			bnl_val = max_pressure if bnl_val > max_pressure else bnl_val
			bnr_val = max_pressure if bnr_val > max_pressure else bnr_val
			bsl_val = max_pressure if bsl_val > max_pressure else bsl_val
			bsr_val = max_pressure if bsr_val > max_pressure else bsr_val

			self.base_neck_left_dofs.pressure_constraint.findData('value').value = str(bnl_val)
			self.base_neck_right_dofs.pressure_constraint.findData('value').value = str(bnr_val)
			self.base_skull_left_dofs.pressure_constraint.findData('value').value = str(bsl_val)
			self.base_skull_right_dofs.pressure_constraint.findData('value').value = str(bsr_val)

			self.update_head_pose()
			plt.ion()
			plt.show()

	def onBeginAnimationStep(self, deltaTime):
		self.deltaTime  += deltaTime
		if self.is_inflated:
			bnl_val = self.base_neck_left_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate
			bnr_val = self.base_neck_right_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate
			bsl_val = self.base_skull_left_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate
			bsr_val = self.base_skull_right_dofs.pressure_constraint.findData('value').value[0][0] + self.growth_rate
			logger.info('inflating base IABs to {} at time {}'.format(bnl_val, self.deltaTime))

		pat_pose = self.patient_dofs.findData('rest_position').value
		x, y, z = [t[0] for t in pat_pose], [t[1] for t in pat_pose], [t[2] for t in pat_pose]
		# logger.debug('len x: {}, y: {}, z: {}: '.format(len(x), len(y), len(z), len(pat_pose)))
		# check to see if patient is above a z
		curr_pat_pose = np.linalg.norm(pat_pose, axis=0)
		if self.first_iter:
			self.thresholds['patient_trans'] = (curr_pat_pose+10000).tolist()
			self.thresholds.update(self.thresholds)
			self.first_iter = False
		logger.debug('curr_pat_pose: {}, \n\t thresh trans {}'.format(curr_pat_pose, self.thresholds['patient_trans']))

		self.base_neck_left_dofs.pressure_constraint.findData('value').value = str(bnl_val)
		self.base_neck_right_dofs.pressure_constraint.findData('value').value = str(bnr_val)
		self.base_skull_left_dofs.pressure_constraint.findData('value').value = str(bsl_val)
		self.base_skull_right_dofs.pressure_constraint.findData('value').value = str(bsr_val)

		if curr_pat_pose[2]>thresholds['patient_trans'][2]:
			#curr_pat_pose[0]>thresholds['patient_trans'][0] and \
		   #curr_pat_pose[1]>thresholds['patient_trans'][1] and \
		   logger.warning('reached max z limit')
		   self.is_inflated = False
		   # bnl_val = max_pressure if bnl_val > max_pressure else bnl_val
		   # bnr_val = max_pressure if bnr_val > max_pressure else bnr_val
		   # bsl_val = max_pressure if bsl_val > max_pressure else bsl_val
		   # bsr_val = max_pressure if bsr_val > max_pressure else bsr_val

		# self.update_head_pose()
		# plt.ion()
		# plt.show()

		return 0;

	def onEndAnimationStep(self, deltaTime):
		sys.stdout.flush()
		return 0;

	def onLoaded(self, node):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def reset(self):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
		# usage e.g.
		if isPressed :
		   print ("Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY))
		return 0;

	def bwdInitGraph(self, node):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def onScriptEvent(self, senderNode, eventName,data):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def onMouseButtonRight(self, mouseX,mouseY,isPressed):
		## usage e.g.
		#if isPressed :
		#    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
		return 0;
