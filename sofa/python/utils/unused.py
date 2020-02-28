
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
'''
