__author__ = "Lekan Ogunmolu"
__description__ = "Lekan generated this file on 02/13/2020"
__examples__ = "See SoftRobotsPlugin Examples  \
                (i)  /sofa/applications/plugins/SoftRobots/docs/\
                examples/component/constraint/SurfacePressureConstraint/SurfacePressureConstraint.pyscn \
                (ii) python/pneunets.py"


import sys
import Sofa
import os
from os.path import join

pwd = os.getcwd()
meshes_dir = join(pwd, '../..', 'ros/srs_traj_opt/patient_description/meshes')
dome_mass = .04 # as measured by newly fabricated soft robot
poisson=0.3   # poisson from pneunet parameters
youngs=18000  # using pneunet parameters

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

        rootNode.createObject('RequiredPlugin', pluginName='IAB', name='IAB')
        rootNode.createObject('RequiredPlugin', pluginName='IABTetra', name='IABTetra')
        rootNode.createObject('RequiredPlugin', name='SofaPython', pluginName='SofaPython')
        rootNode.createObject('RequiredPlugin', name='SoftRobots', pluginName='SoftRobots')
        rootNode.createObject('RequiredPlugin', pluginName='CImgPlugin', name='CImgPlugin')
        rootNode.createObject('RequiredPlugin', name='SofaSparseSolver', pluginName='SofaSparseSolver')
        rootNode.createObject('RequiredPlugin', name='SofaOpenglVisual', pluginName='SofaOpenglVisual')
        rootNode.createObject('RequiredPlugin', name='SofaPreconditioner', pluginName='SofaPreconditioner')
        rootNode.createObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels showBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

        rootNode.createObject('FreeMotionAnimationLoop')
        rootNode.createObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')
        rootNode.createObject('PythonScriptController', filename="diff_kine_controller.py", classname="controller")

        self.DomeHead=rootNode.createChild('DomeHead')
        self.DomeHead.createObject('EulerImplicitSolver', name='cg_odesolver', printLog='false')
        self.DomeHead.createObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', preconditioners='preconditioner', use_precond='true', update_step='1')

        self.DomeHead.createObject('MeshVTKLoader', name='domeHeadVTKLoader', filename='{}/dome/dome.vtu'.format(meshes_dir))
        self.DomeHead.createObject('TetrahedronSetTopologyContainer', src='@domeHeadVTKLoader', name='TetraTopologyContainer') #createTriangleArray='true',
        self.DomeHead.createObject('TetrahedronSetTopologyModifier', name='TetraTopologyModifier')
        self.DomeHead.createObject('TetrahedronSetTopologyAlgorithms', name='TetraTopologyAlgo', template='Vec3d')
        self.DomeHead.createObject('TetrahedronSetGeometryAlgorithms', drawTetrahedra='1', name='TetraGeomAlgo', template='Vec3d')

        self.DomeHead.createObject('MechanicalObject', name='dh_dofs', template='Vec3d', showIndices='false', rx='0', showIndicesScale='4e-5', dz="0") #src='@domeHeadVTKLoader',
        self.DomeHead.createObject('UniformMass', totalMass='{}'.format(dome_mass))
        self.DomeHead.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='{}'.format(poisson),  youngModulus='{}'.format(youngs))

        # this from paraview
        self.DomeHead.createObject('BoxROI', name='boxROI', box='-2.75533 2.74354 -.1597 2.76615 -2.99312 2.99312', drawBoxes='false', doUpdate='0')#, position="@dh_dofs.rest_position", tetrahedra="@TetraTopologyContainer.tetrahedra")
        self.DomeHead.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')

        self.DomeHead.createObject('SparseLDLSolver', name='preconditioner')
        self.DomeHead.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')

        # rootNode.createObject('TetrahedronMooneyRivlinFEMForceField', name='rootFEM', materialName='MooneyRivlinIncompressible', ParameterSet='1000 100', template='Vec3d', poissonRatio='0.45', youngModulus='10000')

        # rootNode/DomeCavity
        self.DomeCavity = self.DomeHead.createChild('DomeCavity')
        self.DomeCavity.createObject('MeshObjLoader', name='cavityLoader', filename='{}/dome/dome_cavity.obj'.format(meshes_dir), triangulate="true")
        self.DomeCavity.createObject('Mesh', src='@cavityLoader', name='cavityTopo')
        self.DomeCavity.createObject('MechanicalObject',  name='domeCavity', rx="90")
        self.DomeCavity.createObject('SurfacePressureConstraint', name='surfaceConstraint', template="Vec3d", value="0.0001", triangles='@cavityTopo.triangles', drawPressure='0', drawScale='0.0002', valueType='pressure') #
        self.DomeCavity.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')# , input='@../dh_dofs',  output='@DomeCavity', template="Vec3d")

        # rootNode/DomeHeadCollis
        self.DomeHeadCollis = self.DomeHead.createChild('DomeHeadCollis')
        self.DomeHeadCollis.createObject('MeshObjLoader', name='domeHeadCollis', filename='{}/dome/dome_cavity.obj'.format(meshes_dir)) #src='@../domeHeadVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
        self.DomeHeadCollis.createObject('Mesh', src='@domeHeadCollis', name='topo')
        self.DomeHeadCollis.createObject('MechanicalObject', name='collisMech', rx="90")
        self.DomeHeadCollis.createObject('Triangle', selfCollision="false")
        self.DomeHeadCollis.createObject('Line',selfCollision="false")
        self.DomeHeadCollis.createObject('Point', selfCollision="false")
        self.DomeHeadCollis.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')# , input='@../dh_dofs',  output='@DomeCavity', template="Vec3d")

        # rootNode/DomeHeadVisu
        self.DomeHeadVisu = self.DomeHead.createChild('DomeHeadVisu')
        self.DomeHeadVisu.createObject('TriangleSetTopologyContainer', name='triContainer')
        self.DomeHeadVisu.createObject('TriangleSetTopologyModifier', name='triMod')
        self.DomeHeadVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
        self.DomeHeadVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
        self.DomeHeadVisu.createObject('Tetra2TriangleTopologicalMapping', name='DHTetra2TriMap', input='@../TetraTopologyContainer', output='@triContainer')
        self.DomeHeadVisu.createObject('OglModel', color='0.3 0.2 0.2 0.6') # template='Vec3d',

        '''
        # Dome Cover
        self.DomeCoverVisu = self.DomeHead.createChild('DomeCoverVisu')
        self.DomeCoverVisu.createObject('MeshSTLLoader', name='domeCoverLoader', filename='{}/dome/cover.stl'.format(meshes_dir))
        self.DomeCoverVisu.createObject('Mesh', src='@domeCoverLoader', name='dome_ring_cav_mesh')
        self.DomeCoverVisu.createObject('MechanicalObject', name='domeCoverVisu')
        self.DomeCoverVisu.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')
        self.DomeCoverVisu.createObject('OglModel', color='0.3 0.5 0.5 0.6')

        # DomeCoverCollis
        self.DomeCoverCollis = self.DomeHead.createChild('DomeCoverCollis')
        self.DomeCoverCollis.createObject('MeshObjLoader', name='domeCoverCollis', filename='{}/dome/cover.obj'.format(meshes_dir)) #src='@../domeHeadVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
        self.DomeCoverCollis.createObject('Mesh', src='@domeCoverCollis', name='topo')
        self.DomeCoverCollis.createObject('MechanicalObject', name='collisMech', rx="-90", dz="0.1" )
        self.DomeCoverCollis.createObject('Triangle', selfCollision="false")
        self.DomeCoverCollis.createObject('Line',selfCollision="false")
        self.DomeCoverCollis.createObject('Point', selfCollision="false")
        self.DomeCoverCollis.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')# , input='@../dh_dofs',  output='@DomeCavity', template="Vec3d")
        '''

        return rootNode;

    def initGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/lex/catkin_ws/src/superchicko/sofa/python/xml_2_scn.py
        return 0;

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
