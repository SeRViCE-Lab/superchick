"""
turntablePython
is based on the scene 
/home/lex/catkin_ws/src/superchicko/sofa/python/../scenes/scene_comps/gantry/turntable.scn
but it uses the SofaPython plugin. 
Further informations on the usage of the plugin can be found in 
sofa/applications/plugins/SofaPython/doc/SofaPython.pdf
To launch the scene, type 
runSofa /home/lex/catkin_ws/src/superchicko/sofa/python/../scenes/scene_comps/gantry/turntablePython.py --argv 123
The sofa python plugin might have to be added in the sofa plugin manager, 
i.e. add the sofa python plugin in runSofa->Edit->PluginManager.
The arguments given after --argv can be used by accessing self.commandLineArguments, e.g. combined with ast.literal_eval to convert a string to a number.

The current file has been written by the python script
/home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
Author of xml_2_scn.py: Christoph PAULUS, christoph.paulus@inria.fr
"""

import sys
import Sofa

class turntable (Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments) : 
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):

        # rootNode
        rootNode.createObject('MeshSTLLoader', scale='5e-6', name='turntable-loader', filename='../../../../ros/srs_traj_opt/couch_description/meshes/truebeam/Assembly4/TurnTable.stl')
        rootNode.createObject('Mesh', src='@turntable-loader', name='turntable-mesh')
        rootNode.createObject('MechanicalObject', src='@turntable-loader', name='turntable-states', template='Vec3d')
        rootNode.createObject('UniformMass', totalMass='30')

        # rootNode/TurnTableVisu
        TurnTableVisu = rootNode.createChild('TurnTableVisu')
        self.TurnTableVisu = TurnTableVisu
        TurnTableVisu.createObject('OglModel', color='grey', src='@../turntable-loader', translation='0 0 0', rotation='0 0 0', name='turntableVisual')

        # rootNode/TurnTableCollis
        TurnTableCollis = rootNode.createChild('TurnTableCollis')
        self.TurnTableCollis = TurnTableCollis
        TurnTableCollis.createObject('MeshTopology', name='meshTopology')
        TurnTableCollis.createObject('MechanicalObject', src='@../turntable-loader', name='turntable-collis-states', template='Vec3d', restScale='1')

        return 0;

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onKeyReleased(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You released a"
        return 0;

    def initGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
        return 0;

    def onKeyPressed(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You pressed control+a"
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        ## usage e.g.
        #if isPressed : 
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
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
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
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


def createScene(rootNode):
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    myturntable = turntable(rootNode,commandLineArguments)
    return 0;