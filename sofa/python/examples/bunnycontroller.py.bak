#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import Sofa
import math



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


    def initGraph(self, root):

        '''
        IABs are so named:
        Convention is top of head is facing the screen. Left is to your left ear as it would be if you were facing the screen

        Bottom IABs: {{bunny, neck_right}, {skull_left, skull_right}}
        Side   IABs: {{left_neck, left_skull}, {right_neck, right_skull}}
        '''
        # print([k, v for k, v in root.get_items()])
        self.root = root
        self.bunny_node=self.root.getChild('bunny')
        self.bunny_cavity = self.bunny_node.getChild('cavity')

        self.centerPosY = 70
        self.centerPosZ = 0
        self.rotAngle = 0

    def onKeyPressed(self,c):
        self.dt = self.root.findData('dt').value
        incr = self.dt*1000.0;
        self.bunny_mech=self.bunny_node.getObject('dh_dofs');
        self.bunny_constraint = self.bunny_cavity.getObject('SurfacePressureConstraint')

        if (c == "+"):
            print('squeezing...')
            pressureValue = self.bunny_constraint.findData('value').value[0][0] + 0.01
            if pressureValue > 1.5:
                pressureValue = 1.5
            self.bunny_constraint.findData('value').value = str(pressureValue)

        if (c == "-"):
            print( 'releasing...')
            pressureValue = self.bunny_constraint.findData('value').value[0][0] - 0.01
            self.bunny_constraint.findData('value').value = str(pressureValue)

        # UP key :
        if ord(c)==19:
            test1 = moveRestPos(self.bunny_mech.rest_position, 3.0, 0.0, 0.0)
            self.bunny_mech.findData('rest_position').value = test1

        # DOWN key : rear
        if ord(c)==21:
            test = moveRestPos(self.bunny_mech.rest_position, -3.0, 0.0, 0.0)
            self.bunny_mech.findData('rest_position').value = test

        # LEFT key : left
        if ord(c)==20:
            dy = 3.0*math.cos(self.rotAngle)
            dz = 3.0*math.sin(self.rotAngle)
            test = moveRestPos(self.bunny_mech.rest_position, 0.0, dy, dz)
            self.bunny_mech.findData('rest_position').value = test
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        # RIGHT key : right
        if ord(c)==18:
            dy = -3.0*math.cos(self.rotAngle)
            dz = -3.0*math.sin(self.rotAngle)
            test = moveRestPos(self.bunny_mech.rest_position, 0.0, dy, dz)
            self.bunny_mech.findData('rest_position').value = test
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        # a key : direct rotation
        if (ord(c) == 65):
            test = rotateRestPos(self.bunny_mech.rest_position, math.pi/16, self.centerPosY,self.centerPosZ)
            self.bunny_mech.findData('rest_position').value = test
            self.rotAngle = self.rotAngle + math.pi/16

        # q key : indirect rotation
        if (ord(c) == 81):
            test = rotateRestPos(self.bunny_mech.rest_position, -math.pi/16, self.centerPosY,self.centerPosZ)
            self.bunny_mech.findData('rest_position').value = test
            self.rotAngle = self.rotAngle - math.pi/16

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        # usage e.g.
        if isPressed :
           print("Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY))
        return 0;

    def onKeyReleased(self, c):
        # usage e.g.
        if c=="A" :
           print( "You released a")
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        # usage e.g.
        if isPressed :
           print( "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta))
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

    def onEndAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
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
           print( "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY))
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

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
        return 0;
