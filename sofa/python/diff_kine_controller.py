#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import Sofa
import math
import sys

move_dist = 20
growth_rate = .5  #was .05
max_pressure = 100 # was 15

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

class Bundle(object):
    def __init__(self, dicko):

        for var, val in dicko.items():
            object.__setattr__(self, var, val)

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

        self.patient = root.getChild('patient')
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

        'get all the dofs associated with each dome in the linkage'
        self.bnl_dofs = self.get_dome_dofs(self.base_neck_left)
        self.bnr_dofs = self.get_dome_dofs(self.base_neck_right)
        self.bsl_dofs = self.get_dome_dofs(self.base_skull_left)
        self.bsr_dofs = self.get_dome_dofs(self.base_skull_right)
        # side domes
        self.sfl_dofs = self.get_dome_dofs(self.side_fore_left)
        self.scl_dofs = self.get_dome_dofs(self.side_chin_left)
        self.sfr_dofs = self.get_dome_dofs(self.side_fore_right)
        self.scr_dofs = self.get_dome_dofs(self.side_chin_right)

    # domes' mechanical states
    def get_dome_dofs(self, node):
        'dof name shall be in the form patient or base_neck etc'
        dh_dofs = node.getObject('dh_dofs')  # dome head
        # dh_collis_dofs = node.getObject('dh_collis_dofs')
        # cavity
        cav_dofs = node.getObject('dome_cav_dofs')
        cav_collis_dofs = node.getObject('dome_cav_collis_dofs')
        # dome cover back
        cover_dofs = node.getObject('dome_cover_dofs')
        cover_collis_dofs = node.getObject('dome_cover_collis_dofs')

        return Bundle(dict(dh_dofs=dh_dofs, cav_dofs=cav_dofs,
                        cav_collis_dofs=cav_collis_dofs,
                        cover_dofs=cover_dofs,
                        cover_collis_dofs=cover_collis_dofs))

    def onKeyPressed(self,c):
        self.dt = self.root.findData('dt').value
        incr = self.dt*1000.0;
        self.neck_left_mech=self.base_neck_left.getObject('dh_dofs');
        self.neck_left_constraint = self.neck_left_cavity.getObject('SurfacePressureConstraint')

        if (c == "+"):
            pressureValue = self.neck_left_constraint.findData('value').value[0][0] + growth_rate
            # print('Current pressure value: ', pressureValue, see_pose(self.neck_left_mech.rest_position))
            if pressureValue > max_pressure:
                pressureValue = max_pressure
            self.neck_left_constraint.findData('value').value = str(pressureValue)
            print('squeezing... at {}'.format(pressureValue))

        if (c == "-"):
            # print( 'releasing...')
            pressureValue = self.neck_left_constraint.findData('value').value[0][0] - growth_rate
            self.neck_left_constraint.findData('value').value = str(pressureValue)
            print('releasing... at {}'.format(pressureValue))

        # UP key :
        if ord(c)==19:
            # print('moving along +x by {}'.format(move_dist))
            test1 = moveRestPos(self.neck_left_mech.rest_position, move_dist, 0.0, 0.0)
            self.neck_left_mech.findData('rest_position').value = test1

        # DOWN key : rear
        if ord(c)==21:
            # print('moving along -x by {}'.format(move_dist))
            test = moveRestPos(self.neck_left_mech.rest_position, -move_dist, 0.0, 0.0)
            self.neck_left_mech.findData('rest_position').value = test

        # LEFT key : left
        if ord(c)==20:
            dy = 3.0*math.cos(self.rotAngle)
            dz = 3.0*math.sin(self.rotAngle)
            test = moveRestPos(self.neck_left_mech.rest_position, 0.0, dy, dz)
            self.neck_left_mech.findData('rest_position').value = test
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        # RIGHT key : right
        if ord(c)==18:
            dy = -3.0*math.cos(self.rotAngle)
            dz = -3.0*math.sin(self.rotAngle)
            test = moveRestPos(self.neck_left_mech.rest_position, 0.0, dy, dz)
            self.neck_left_mech.findData('rest_position').value = test
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        # a key : direct rotation
        if (ord(c) == 65):
            test = rotateRestPos(self.neck_left_mech.rest_position, math.pi/16, self.centerPosY,self.centerPosZ)
            self.neck_left_mech.findData('rest_position').value = test
            self.rotAngle = self.rotAngle + math.pi/16

        # q key : indirect rotation
        if (ord(c) == 81):
            test = rotateRestPos(self.neck_left_mech.rest_position, -math.pi/16, self.centerPosY,self.centerPosZ)
            self.neck_left_mech.findData('rest_position').value = test
            self.rotAngle = self.rotAngle - math.pi/16

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

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        # usage e.g.
        # if isPressed :
        #    print ("Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta))
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
