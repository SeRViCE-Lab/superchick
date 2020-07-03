#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
	Use this to execute the differential kinematics
	controller in our kinecontrol paper.
'''
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

# https://www.sofa-framework.org/community/forum/topic/get-the-position-value-from-a-mechanicalobject-point-in-python/
def moveRestPos(rest_pos, pose):
	str_out = ' '
	dx, dy, dz = pose
	for i in range(0,len(rest_pos)) :
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

		self.root = root

		dome_all_dofs 		= self.get_dome_dofs(self.root)
		self.dh_dofs		= dome_all_dofs.dh_dofs
		self.cav_dofs		= dome_all_dofs.cav_dofs
		self.cover_dofs		= dome_all_dofs.cover_dofs

	# domes' mechanical states
	def get_dome_dofs(self, node):
		'here node is root'
		domehead = node.getChild('DomeHead')
		dh_dofs = domehead.getObject('dh_dofs')

		cav_node = domehead.getChild('DomeCavity')
		cav_dofs = cav_node.getObject('dome_cav_dofs')

		cover_node = domehead.getChild('DomeCover')
		cover_dofs = cover_node.getObject('dome_cover_dofs')

		cover_collis_node = domehead.getChild('DomeCoverCollis')
		cover_collis_dofs = cover_collis_node.getObject('dome_cover_collis_dofs')

		return Bundle(dict(dh_dofs=dh_dofs,
						   cav_dofs=cav_dofs,
						   cover_dofs=cover_dofs
						))

	def bwdInitGraph(self,node):
		# find the position at the end of the shape (which has the biggest x coordinate)
		dh_dofs = self.get_dome_dofs(self.root).dh_dofs.position
		max_x, max_y, max_z = 0, 0, 0
		max_idx_x, max_idx_y, max_idx_z = 0, 0, 0

		for i in range(len(dh_dofs)):
			if dh_dofs[i][0] > max_x:
				max_idx_x = i
				max_x = dh_dofs[i][0]
			if dh_dofs[i][1] > max_y:
				max_idx_y = i
				max_y = dh_dofs[i][1]
			if dh_dofs[i][2] > max_z:
				max_idx_z = i
				max_z = dh_dofs[i][2]

		self.max_vals = Bundle(dict(max_x=max_x, max_y=max_y, max_z=max_z))
		print('dh trans [x,y,z] {}, {}, {}'.format(max_x, max_y, max_z))
		return 0

	def run_traj_plotter(self):
		if self.is_chart_updated:
			self.traj_plotter.update(self.data)
			# time.sleep(.11)
		self.is_chart_updated = False

		return 0

	def deform_positive(self, dofs):
		print('dome head dofs: ', dofs.position)


	def onBeginAnimationStep(self, deltaTime):
		deltaTime  += deltaTime
		# obtain associated dofs and cavity dofs

		while(deltaTime < 2):
			self.deform_positive(self.dh_dofs)

		return 0;

	def onEndAnimationStep(self, deltaTime):
		sys.stdout.flush()
		#access the 'position' state vector
		self.bwdInitGraph(self.root)
		return 0;

	def onKeyPressed(self, c):
		self.dt = self.root.findData('dt').value
		incr = self.dt*1000.0
		self.dh_dofs = self.get_dome_dofs(self.root).dh_dofs
		# self.dh_dofs		= dome_all_dofs.dh_dofs

		if (ord(c)==19): # UP Key
			print("expanding ...")
			test = moveRestPos(self.dh_dofs.position, (300.0, 300.0, 300.0))
			self.dh_dofs.findData('position').value = test

		if (ord(c)==21): # DOWN Key
			print("retracting ...")
			test = moveRestPos(self.dh_dofs.position, (-300.0, -300.0, -300.0))
			self.dh_dofs.findData('position').value = test

		self.bwdInitGraph(self.root)

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
