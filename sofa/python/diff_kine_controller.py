#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import Sofa
import math
import sys
import time
import datetime
import numpy as np
from utils import *

# import matplotlib as mpl
# mpl.use('qt5agg')
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec

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

def see_pose(pos):
	str_out = ' '
	for i in xrange(0,len(pos)) :
		str_out= str_out + ' ' + str(pos[i][0])
		str_out= str_out + ' ' + str(pos[i][1])
		str_out= str_out + ' ' + str(pos[i][2])
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

		# IABs are so named:
		# Convention is top of head is facing the screen. Left is to your left ear as it would be if you were facing the screen
		#
		# Bottom IABs: {{neck_left, neck_right},{skull_left, skull_right}}
		# Side   IABs: {{fore_left, chin_left}, {fore_right, chin_right}}

		starttime = datetime.datetime.now()
		begintime = time.time()

		self._fig = plt.figure()
		self._gs = gridspec.GridSpec(1,1) # rows cols
		self.traj_plotter = HeadTrajPlotter(self._fig, self._gs[0]) # subplot in gridspec

		self.root = root

		self.patient = root.getChild('patient')
		self.patient_dofs = self.patient.getObject('patient_dofs')
		# get base IABs
		self.base_neck_left=self.root.getChild('base_neck_left')
		self.base_neck_right=self.root.getChild('base_neck_right')
		self.base_skull_left=self.root.getChild('base_skull_left')
		self.base_skull_right=self.root.getChild('base_skull_right')
		# get side IABs
		self.side_fore_left=self.root.getChild('side_fore_left')
		self.side_chin_left=self.root.getChild('side_chin_left')
		self.side_fore_right=self.root.getChild('side_fore_right')
		self.side_chin_right=self.root.getChild('side_chin_right')
		# obtain associated dofs and cavity dofs
		self.base_neck_left_dofs = self.get_dome_dofs(self.base_neck_left)
		self.base_neck_right_dofs = self.get_dome_dofs(self.base_neck_right)
		self.base_skull_left_dofs = self.get_dome_dofs(self.base_skull_left)
		self.base_skull_right_dofs = self.get_dome_dofs(self.base_skull_right)

		self.is_updated = False

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
		if self.is_updated:
			self.traj_plotter.update(self.data)
			# time.sleep(.11)
		self.is_updated = False

	def update_head_pose(self):
		rest_pose = self.patient_dofs.findData('rest_position').value
		# rest pose is a lisrt
		x = [t[0] for t in rest_pose]
		y = [t[1] for t in rest_pose]
		z = [t[2] for t in rest_pose]
		# use 2-norm of x, y, and z
		self.data = np.linalg.norm(np.c_[x, y, z], axis=0)
		self.is_updated = True


	def onKeyPressed(self,c):
		self.dt = self.root.findData('dt').value
		incr = self.dt*1000.0;

		move_dist = (0, .40, 0)
		growth_rate = .5  #was .05
		max_pressure = 100 # was 15

		if (c == "+"):
			print(' raising head using base IABs')
			bnl_val = self.base_neck_left_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate
			bnr_val = self.base_neck_right_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate
			bsl_val = self.base_skull_left_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate
			bsr_val = self.base_skull_right_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate

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
			bnl_val = self.base_neck_left_dofs.pressure_constraint.findData('value').value[0][0] - growth_rate
			bnr_val = self.base_neck_right_dofs.pressure_constraint.findData('value').value[0][0] - growth_rate
			bsl_val = self.base_skull_left_dofs.pressure_constraint.findData('value').value[0][0] - growth_rate
			bsr_val = self.base_skull_right_dofs.pressure_constraint.findData('value').value[0][0] - growth_rate

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
		'''
		# UP key :
		if ord(c)==19:
			bnl_mv = moveRestPos(self.base_neck_left.rest_position, move_dist)
			bnr_mv = moveRestPos(self.base_neck_right.rest_position, move_dist)
			bsl_mv = moveRestPos(self.base_skull_left.rest_position, move_dist)
			bsr_mv = moveRestPos(self.base_skull_right.rest_position, move_dist)
			self.base_neck_left.findData('rest_position').value = bnl_mv
			self.base_neck_left.findData('rest_position').value = bnr_mv
			self.base_neck_left.findData('rest_position').value = bsl_mv
			self.base_neck_left.findData('rest_position').value = bsr_mv

			self.update_head_pose()

		# DOWN key : rear
		if ord(c)==21:
			bnl_mv = moveRestPos(self.base_neck_left.rest_position, -move_dist)
			bnr_mv = moveRestPos(self.base_neck_right.rest_position, -move_dist)
			bsl_mv = moveRestPos(self.base_skull_left.rest_position, -move_dist)
			bsr_mv = moveRestPos(self.base_skull_right.rest_position, -move_dist)
			self.base_neck_left.findData('rest_position').value = bnl_mv
			self.base_neck_left.findData('rest_position').value = bnr_mv
			self.base_neck_left.findData('rest_position').value = bsl_mv
			self.base_neck_left.findData('rest_position').value = bsr_mv

			self.update_head_pose()

		# LEFT key : left
		if ord(c)==20:
			mv_left = (0, 10, 10)
			test = moveRestPos(self.base_neck_left.rest_position, 0.0, dy, dz)
			self.base_neck_left.findData('rest_position').value = test

		# RIGHT key : right
		if ord(c)==18:
			dy = -3.0*math.cos(self.rotAngle)
			dz = -3.0*math.sin(self.rotAngle)
			test = moveRestPos(self.neck_left_mech.rest_position, 0.0, dy, dz)
			self.neck_left_mech.findData('rest_position').value = test

		# a key : direct rotation
		if (ord(c) == 65):
			test = rotateRestPos(self.neck_left_mech.rest_position, math.pi/16, self.centerPosY,self.centerPosZ)
			self.neck_left_mech.findData('rest_position').value = test

		# q key : indirect rotation
		if (ord(c) == 81):
			test = rotateRestPos(self.neck_left_mech.rest_position, -math.pi/16, self.centerPosY,self.centerPosZ)
			self.neck_left_mech.findData('rest_position').value = test
			self.rotAngle = self.rotAngle - math.pi/16
	'''

	def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
		# usage e.g.
		if isPressed :
		   print ("Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY))
		return 0;

	def onKeyReleased(self, c):
		# usage e.g.
		if c=="A" :
		   print ("You released a")
		return 0;

	def onMouseWheel(self, mouseX,mouseY,wheelDelta, isPressed):
		# usage e.g.
		if isPressed :
		   print ("Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta))
		return 0;

	def storeResetState(self):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def cleanup(self):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def onGUIEvent(self, strControlID,valueName,strValue):
		## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
		return 0;

	def onBeginAnimationStep(self, deltaTime):
		self.time += deltaTime
		print(' raising head using base IABs: bnl_val: {} bnr_val: {} bsl_val: {} bsr_val')
		bnl_val = self.base_neck_left_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate
		bnr_val = self.base_neck_right_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate
		bsl_val = self.base_skull_left_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate
		bsr_val = self.base_skull_right_dofs.pressure_constraint.findData('value').value[0][0] + growth_rate

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
