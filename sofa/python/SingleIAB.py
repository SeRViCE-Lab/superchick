__author__ = "Lekan Ogunmolu"
__description__ = "Lekan generated this file on 02/13/2020"

"""
dome_test
is based on the scene
/home/lex/catkin_ws/src/superchicko/sofa/python/../scenes/dome_test.scn
but it uses the SofaPython plugin.
Further informations on the usage of the plugin can be found in
sofa/applications/plugins/SofaPython/doc/SofaPython.pdf
To launch the scene, type
runSofa /home/lex/catkin_ws/src/superchicko/sofa/python/dome_test.py --argv 123
The sofa python plugin might have to be added in the sofa plugin manager,
i.e. add the sofa python plugin in runSofa->Edit->PluginManager.
The arguments given after --argv can be used by accessing self.commandLineArguments, e.g. combined with ast.literal_eval to convert a string to a number.

The current file has been written by the python script
/home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
Author of xml_2_scn.py: Christoph PAULUS, christoph.paulus@inria.fr
"""

import sys
import Sofa
import os

# ros_dir = os.getcwd() +

class dome_test (Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments) :
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):

        # rootNode
        rootNode.createObject('APIVersion', level='19.06')
        rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
        rootNode.createObject('OglSceneFrame', style='Arrows', alignment='BottomLeft')
        rootNode.createObject('RequiredPlugin', pluginName='CImgPlugin', name='CImgPlugin')
        rootNode.createObject('RequiredPlugin', name='SofaOpenglVisual', pluginName='SofaOpenglVisual')
        rootNode.createObject('RequiredPlugin', name='SofaPython', pluginName='SofaPython')
        rootNode.createObject('RequiredPlugin', name='SoftRobots', pluginName='SoftRobots')
        rootNode.createObject('RequiredPlugin', name='SofaMiscCollision', pluginName='SofaMiscCollision')
        rootNode.createObject('RequiredPlugin', pluginName='IAB', name='IAB')
        rootNode.createObject('RequiredPlugin', pluginName='IABTetra', name='IABTetra')
        rootNode.createObject('VisualStyle', displayFlags='showForceFields showBehaviorModels')
        rootNode.createObject('DefaultPipeline', verbose='0')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('DefaultContactManager', response='default')
        rootNode.createObject('MinProximityIntersection', contactDistance='0.5', alarmDistance='0.8', name='Proximity')
        rootNode.createObject('DefaultCollisionGroupManager')
        rootNode.createObject('DefaultAnimationLoop')
        rootNode.createObject('EulerImplicitSolver', name='cg_odesolver', printLog='false')
        rootNode.createObject('CGLinearSolver', threshold='1.0e-9', tolerance='1.0e-9', name='linear solver', iterations='25')
        rootNode.createObject('MeshVTKLoader', name='domeHeadVTKLoader', filename='../../ros/srs_traj_opt/patient_description/meshes/dome/dome.vtu')
        rootNode.createObject('MechanicalObject', src='@domeHeadVTKLoader', dz='0', name='dh_dofs', template='Vec3d', showIndices='false', rx='0', showIndicesScale='4e-5')
        rootNode.createObject('Mesh', name="dh_mesh", src='@domeHeadVTKLoader')
        rootNode.createObject('TetrahedronSetTopologyContainer', src='@domeHeadVTKLoader', createTriangleArray='true', name='TetraTopologyContainer')
        rootNode.createObject('TetrahedronSetTopologyModifier', name='TetraTopologyModifier')
        rootNode.createObject('TetrahedronSetGeometryAlgorithms', drawTetrahedra='1', name='TetraGeomAlgo', template='Vec3d')
        rootNode.createObject('TetrahedronSetTopologyAlgorithms', name='TetraTopologyAlgo', template='Vec3d')
        rootNode.createObject('DiagonalMass', massDensity='1', name='computed using mass density')
        rootNode.createObject('TetrahedronMooneyRivlinFEMForceField', name='rootFEM', materialName='MooneyRivlinIncompressible', ParameterSet='1000 100', template='Vec3d', poissonRatio='0.45', youngModulus='10000')
        rootNode.createObject('MeshSTLLoader', name='domeHeadSTLLoader', filename='../../ros/srs_traj_opt/patient_description/meshes/dome/dome.stl')

        # rootNode/DomeHeadVisu
        DomeHeadVisu = rootNode.createChild('DomeHeadVisu')
        self.DomeHeadVisu = DomeHeadVisu
        DomeHeadVisu.tags = 'Visual'
        DomeHeadVisu.gravity = '0 -9.81 0'
        DomeHeadVisu.createObject('OglModel', src='@../domeHeadSTLLoader', scale='1', name='DomeHeadVisual', color='', rx='0', ry='0', rz='0', dz='0', dx='0', dy='0')
        DomeHeadVisu.createObject('TriangleSetTopologyContainer', name='triContainer')
        DomeHeadVisu.createObject('TriangleSetTopologyModifier', name='triMod')
        DomeHeadVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
        DomeHeadVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
        DomeHeadVisu.createObject('Tetra2TriangleTopologicalMapping', input='@../TetraTopologyContainer', name='Tetra2TriMap', output='@triContainer')
        DomeHeadVisu.createObject('BarycentricMapping', input='@../dh_dofs', name='visual mapping', output='@DomeHeadVisual')

    #     # rootNode/DomeCavity
    #     DomeCavity = rootNode.createChild('DomeCavity')
    #     self.DomeCavity = DomeCavity
    #     DomeCavity.tags = 'Visual'
    #     DomeCavity.gravity = '0 -9.81 0'
    #     DomeCavity.createObject('Mesh', src='@../domeHeadSTLLoader', name='cavityLoader')
    #     DomeCavity.createObject('MechanicalObject', src='@cavityLoader', scale='0.5', name='cavityLoaderMech', rx='0', showIndicesScale='4e-5', dz='0', template='Vec3d', showIndices='false')
    #     DomeCavity.createObject('SurfacePressureConstraint', triangles='@cavityLoader.triangles', valueType='pressure', name='surfaceConstraint', value='.02')
    #     DomeCavity.createObject('BarycentricMapping', input='@../dh_dofs', mapMasses='False', output='@DomeCavity', name='DomeCavityBaryMapping', mapForces='False')
    #
    #     # rootNode/DomeHeadCollis
    #     DomeHeadCollis = rootNode.createChild('DomeHeadCollis')
    #     self.DomeHeadCollis = DomeHeadCollis
    #     DomeHeadCollis.gravity = '0 -9.81 0'
    #     DomeHeadCollis.createObject('MechanicalObject', src='@../domeHeadVTKLoader', scale='1', name='dome-collis-states', rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
    #     DomeHeadCollis.createObject('BarycentricMapping', input='@../dh_dofs', name='dome head collision mapping', output='@DomeHeadCollis')
    #     rootNode.createObject('EulerImplicitSolver', name='domeRing_cg_odesolver', printLog='false')
    #     rootNode.createObject('CGLinearSolver', threshold='1.0e-9', tolerance='1.0e-9', name='linear solver_dome_ring', iterations='25')
    #     rootNode.createObject('MeshVTKLoader', name='vtkDomeRingLoader', filename='../../ros/srs_traj_opt/patient_description/meshes/dome/dome_ring.vtu')
    #     rootNode.createObject('Mesh', name='dome_ring_mesh', src='@vtkDomeRingLoader')
    #     rootNode.createObject('MechanicalObject', src='@vtkDomeRingLoader', dz='0', name='dr_dofs', template='Vec3d', showIndices='false', rx='0', showIndicesScale='4e-5')
    #     rootNode.createObject('TetrahedronSetTopologyContainer', src='@vtkDomeRingLoader', name='TetraTopologyContainer_dome_ring')
    #     rootNode.createObject('TetrahedronSetGeometryAlgorithms', drawTetrahedra='1', name='domeRing_TetraGeomAlgo', template='Vec3d')
    #     rootNode.createObject('TetrahedronSetTopologyAlgorithms', name='domeRing_TetraTopologyAlgo', template='Vec3d')
    #     rootNode.createObject('TetrahedronSetTopologyModifier', name='domeRing_TetraTopologyModifier')
    #     rootNode.createObject('DiagonalMass', massDensity='1', name='computed using mass domeRing_density')
    #     rootNode.createObject('TetrahedronMooneyRivlinFEMForceField', name="dh_fem", materialName='MooneyRivlinIncompressible', ParameterSet='1000 100', template='Vec3d', poissonRatio='0.45', youngModulus='10000')
    #     rootNode.createObject('MeshSTLLoader', name='domeRingSTLLoader', filename='../../ros/srs_traj_opt/patient_description/meshes/dome/dome_ring.stl')
    #
    #     # rootNode/DomeRingVisu
    #     DomeRingVisu = rootNode.createChild('DomeRingVisu')
    #     self.DomeRingVisu = DomeRingVisu
    #     DomeRingVisu.tags = 'Visual'
    #     DomeRingVisu.gravity = '0 -9.81 0'
    #     DomeRingVisu.createObject('OglModel', src='@../domeRingSTLLoader', scale='1', name='DomeRingVisual', color='', rx='0', ry='0', rz='0', dz='0', dx='0', dy='0')
    #     DomeRingVisu.createObject('BarycentricMapping', input='@../dr_dofs', name='visual mapping', output='@DomeRingVisual')
    #
    #     # rootNode/DomeRingCollis
    #     DomeRingCollis = rootNode.createChild('DomeRingCollis')
    #     self.DomeRingCollis = DomeRingCollis
    #     DomeRingCollis.gravity = '0 -9.81 0'
    #     DomeRingCollis.createObject('MechanicalObject', src='@../vtkDomeRingLoader', scale='1', name='dome-ringcollis-states', rx='0', ry='0', rz='0', dz='0', template='Vec3d')
    #     DomeRingCollis.createObject('Mesh', src='@../vtkDomeRingLoader', name='DomeRingMesh')
    #     DomeRingCollis.createObject('BarycentricMapping', input='@../dr_dofs', name='collis mapping', output='@DomeRingCollis')
    #
    #     # rootNode/DomeRingCavity
    #     DomeRingCavity = rootNode.createChild('DomeRingCavity')
    #     self.DomeRingCavity = DomeRingCavity
    #     DomeRingCavity.tags = 'Visual'
    #     DomeRingCavity.gravity = '0 -9.81 0'
    #     DomeRingCavity.createObject('Mesh', src='@../domeRingSTLLoader', name='domeRingCavityLoader')
    #     DomeRingCavity.createObject('MechanicalObject', src='@domeRingCavityLoader', dz='0', name='domeRingCavityLoaderMech', template='Vec3d', showIndices='false', rx='0', showIndicesScale='4e-5')
    #     DomeRingCavity.createObject('SurfacePressureConstraint', triangles='@domeRingCavityLoader.triangles', valueType='pressure', name='surfaceConstraint_dr', value='.02')
    #     DomeRingCavity.createObject('BarycentricMapping', input='@../dr_dofs', mapMasses='False', output='@DomeRingCavity', name='DomeRingCavityBaryMapping', mapForces='False')
    #     rootNode.createObject('EulerImplicitSolver', name='cg_odesolver_ring', printLog='false')
    #     rootNode.createObject('CGLinearSolver', threshold='1.0e-9', tolerance='1.0e-9', name='linear solver domecav', iterations='25')
    #     rootNode.createObject('MeshSTLLoader', name='dome_cover_loader', filename='../../ros/srs_traj_opt/patient_description/meshes/dome/dome_cover.stl')
    #     rootNode.createObject('MechanicalObject', src='@dome_cover_loader', dz='0', name='dofs', template='Rigid3d', showIndices='false', rx='0', showIndicesScale='4e-5')
    #     # rootNode.createObject('Mesh', name='dome_ring_cav_mesh', src='@dome_cover_loader')
    #
    #     # rootNode/DomeCoverVisu
    #     DomeCoverVisu = rootNode.createChild('DomeCoverVisu')
    #     self.DomeCoverVisu = DomeCoverVisu
    #     DomeCoverVisu.createObject('OglModel', src='@../dome_cover_loader', scale='1', name='DomeCoverVisual', rx='0', ry='0', rz='0', dz='0', dx='0', dy='0')
    #     DomeCoverVisu.createObject('RigidMapping', input='@../dofs', output='@DomeCoverVisual')
    #
    #     # rootNode/DomeCoverCollis
    #     DomeCoverCollis = rootNode.createChild('DomeCoverCollis')
    #     self.DomeCoverCollis = DomeCoverCollis
    #     DomeCoverCollis.createObject('MechanicalObject', src='@../dome_cover_loader', scale='1', name='dome-cover-collis-states', rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
    #
    #     return 0;
    #
    # def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
    #     ## usage e.g.
    #     #if isPressed :
    #     #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
    #     return 0;
    #
    # def onKeyReleased(self, c):
    #     ## usage e.g.
    #     #if c=="A" :
    #     #    print "You released a"
    #     return 0;
    #
    # def initGraph(self, node):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def onKeyPressed(self, c):
    #     ## usage e.g.
    #     #if c=="A" :
    #     #    print "You pressed control+a"
    #     return 0;
    #
    # def onMouseWheel(self, mouseX,mouseY,wheelDelta):
    #     ## usage e.g.
    #     #if isPressed :
    #     #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
    #     return 0;
    #
    # def storeResetState(self):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def cleanup(self):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def onGUIEvent(self, strControlID,valueName,strValue):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def onEndAnimationStep(self, deltaTime):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def onLoaded(self, node):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def reset(self):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
    #     ## usage e.g.
    #     #if isPressed :
    #     #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
    #     return 0;
    #
    # def bwdInitGraph(self, node):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def onScriptEvent(self, senderNode, eventName,data):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;
    #
    # def onMouseButtonRight(self, mouseX,mouseY,isPressed):
    #     ## usage e.g.
    #     #if isPressed :
    #     #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
    #     return 0;
    #
    # def onBeginAnimationStep(self, deltaTime):
    #     ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
    #     return 0;


def createScene(rootNode):
    rootNode.findData('dt').value = '0.005'
    rootNode.findData('gravity').value = '0 -9 0'
    try :
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    mydome_test = dome_test(rootNode,commandLineArguments)
    return 0;
