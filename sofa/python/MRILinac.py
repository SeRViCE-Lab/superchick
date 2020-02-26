__author__ = "Lekan Ogunmolu"
__description__ = "Lekan generated this file on 02/13/2020"
__examples__ = "See SoftRobotsPlugin Examples  \
                (i)  /sofa/applications/plugins/SoftRobots/docs/\
                examples/component/constraint/SurfacePressureConstraint/SurfacePressureConstraint.pyscn \
                (ii) python/pneunets.py"

import Sofa
from utils import *
from config import *

class dome_test (Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments) :
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):
        rootNode.createObject('APIVersion', level='19.06')
        # by default, the gravity is defined as "0 -9.81 0". redefined below
        rootNode.findData('gravity').value='0 0 -9810'; # define gravity is along z in mm/sec
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
        # rootNode.createObject('PythonScriptController', filename="diff_kine_controller.py", classname="controller")

        # Patient
        patient = rootNode.createChild('patient')
        patient.createObject('EulerImplicit', name='odesolver')
        patient.createObject('SparseLDLSolver', name='linearSolver')
        patient.createObject('MeshObjLoader', name='patient_loader', filename='{}/patient.obj'.format(dometributes['meshes_dir']))
        patient.createObject('MechanicalObject', src='@patient_loader', name='patient-states', template='Vec3d', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])
        patient.createObject('UniformMass', totalMass='95')
        patient.createObject('UncoupledConstraintCorrection')
        patientCollis = patient.createChild('patientCollis')
        patientCollis.createObject('Mesh', src="@../patient_loader")
        patientCollis.createObject('MechanicalObject', name='patient_dofs', template='Vec3d', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])
        patientVisu = patient.createChild('patientVisu')
        patientVisu.createObject('OglModel', src='@../patient_loader', name='patientVisual', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])

        # IABs
        base_neck_left = dome_maker(rootNode, 'base_neck_left')
        base_neck_right = dome_maker(rootNode, 'base_neck_right')
        base_skull_left = dome_maker(rootNode, 'base_skull_left')
        base_skull_right = dome_maker(rootNode, 'base_skull_right')

        side_fore_left = dome_maker(rootNode, 'side_fore_left')
        side_chin_left = dome_maker(rootNode, 'side_chin_left')
        # side_fore_right = dome_maker(rootNode, 'side_fore_right')
        # side_chin_right = dome_maker(rootNode, 'side_chin_right')
        # side_fore_right
        dome_name = 'side_fore_right'
        dome=rootNode.createChild(dome_name)
        dome.createObject('EulerImplicitSolver', name='{}_odesolver'.format(dome_name), printLog='false')
        dome.createObject('SparseLDLSolver', name='{}_linearSolver'.format(dome_name))
        dome.createObject('MeshVTKLoader', name='{}_loader'.format(dome_name), filename='{}/dome/dome.vtu'.format(dometributes['meshes_dir']),rotation='0 0 90')
        dome.createObject('TetrahedronSetTopologyContainer', src='@{}_loader'.format(dome_name), name='TetraTopologyContainer') #createTriangleArray='true',
        dome.createObject('TetrahedronSetTopologyModifier', name='TetraTopologyModifier')
        dome.createObject('TetrahedronSetTopologyAlgorithms', name='TetraTopologyAlgo', template='Vec3d')
        dome.createObject('TetrahedronSetGeometryAlgorithms', drawTetrahedra='1', name='TetraGeomAlgo', template='Vec3d')
        dome.createObject('MechanicalObject', name='dh_dofs', template='Vec3d', showIndices='false', showIndicesScale='4e-5',\
                                     scale=dometributes['scale'], translation=dometributes['trans'][dome_name], rx=dometributes['rot'][dome_name][0])
        dome.createObject('UniformMass', totalMass='{}'.format(dometributes['mass']))
        dome.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='{}'.format(dometributes['poisson']),\
                                    youngModulus='{}'.format(dometributes['youngs']))
        # stiff layer identified indices from paraview
        dome.createObject('BoxROI', name='boxROI', box='-2.75533 2.74354 -.1597 2.76615 -2.99312 2.99312', drawBoxes='true', doUpdate='1')#, position="@dh_dofs.rest_position", tetrahedra="@TetraTopologyContainer.tetrahedra")
        dome.createObject('BoxROI', name='boxROISubTopo', box='-6.75533 4.74354 -4.7597 4.76615 -3.99312 3.99312', drawBoxes='false', doUpdate='0')#, position="@dh_dofs.rest_position", tetrahedra="@TetraTopologyContainer.tetrahedra")
        # this defines the boundary condition which creates springs between the current position of the body and its initial position
        dome.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e5', angularStiffness='1e5') # stiffness was 1e12 after the pneunets demo
        dome.createObject('SparseLDLSolver', name='preconditioner')
        domeSubTopo = dome.createChild('DomeHeadSubTopo')
        domeSubTopo.createObject('TetrahedronSetTopologyContainer', position='@../{}_loader.position'.format(dome_name), tetrahedra="@boxROISubTopo.tetrahedraInROI", name='container')
        domeSubTopo.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=dometributes['poisson'],
                                            youngModulus=str(dometributes['youngsmodstiff'] - dometributes['youngsmod']))
        domeCavity = dome.createChild('DomeCavity')
        domeCavity.createObject('MeshObjLoader', name='cavityLoader', filename='{}/dome/dome_cavity.obj'.format(dometributes['meshes_dir']), triangulate="true")
        domeCavity.createObject('Mesh', src='@cavityLoader', name='cavityTopo')
        domeCavity.createObject('MechanicalObject',  name='domeCavity', scale=dometributes['scale'], translation=dometributes['trans'][dome_name], rotation='0 0 90')
        domeCavity.createObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template="Vec3d", value=dometributes['pressureConsVal'], triangles='@cavityTopo.triangles', drawPressure='0', drawScale='0.0002', valueType='pressure') #
        domeCavity.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')
        # rootNode/DomeHeadCollis
        # domeCollis = dome.createChild('DomeHeadCollis')
        # domeCollis.createObject('MeshObjLoader', name='domeCollisLoader', filename='{}/dome/dome_cavity.obj'.format(dometributes['meshes_dir'])) #src='@../domeVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
        # domeCollis.createObject('Mesh', src='@domeCollisLoader', name='topo')
        # domeCollis.createObject('MechanicalObject', name='collisMech',  scale=dometributes['scale'], translation=dometributes['trans'][dome_name], rotation='0 0 90')
        # domeCollis.createObject('Triangle', selfCollision="false")
        # domeCollis.createObject('Line',selfCollision="false")
        # domeCollis.createObject('Point', selfCollision="false")
        # domeCollis.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')
        # domeVisu = dome.createChild('DomeHeadVisu')
        # domeVisu.createObject('MeshObjLoader', name='domeVisuLoader', filename='{}/dome/dome.obj'.format(dometributes['meshes_dir'])) #src='@../domeVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
        # domeVisu.createObject('OglModel',  color='0.3 0.2 0.2 0.6') # name='@domeVisuLoader', template='Vec3d', src="@../domeCollisLoader", #dx="20", dy="-80", dz="10", rx="-90",
        # domeVisu.createObject('BarycentricMapping', name='mapping')#, mapForces='false', mapMasses='false')
        # # Dome Cover
        # domeCoverVisu = dome.createChild('DomeCoverVisu')
        # domeCoverVisu.createObject('MeshSTLLoader', name='domeCoverLoader', filename='{}/dome/cover.stl'.format(dometributes['meshes_dir']))
        # domeCoverVisu.createObject('MechanicalObject', name='domeCoverVisu',  scale=dometributes['scale'], \
        #                             translation=dometributes['trans'][dome_name], rotation='0 0 90')
        # domeCoverVisu.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')
        # domeCoverVisu.createObject('OglModel', color='0.3 0.5 0.5 0.6')
        # domeCoverVisu.createObject('OglModel', color='0.3 0.5 0.5 0.6')
        # # DomeCoverCollis
        # domeCoverCollis = dome.createChild('DomeCoverCollis')
        # domeCoverCollis.createObject('MeshObjLoader', name='domeCoverCollis', filename='{}/dome/cover.obj'.format(dometributes['meshes_dir'])) #src='@../domeHeadVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
        # domeCoverCollis.createObject('Mesh', src='@domeCoverCollis', name='topo')
        # if ('side' and 'left') in dome_name:
        #     domeCoverCollis.createObject('MechanicalObject',  name='domeCavity', scale=dometributes['scale'], translation=dometributes['trans'][dome_name], rx="90")
        # elif 'base' and 'left' in dome_name:
        #     domeCoverCollis.createObject('MechanicalObject',  name='domeCavity', scale=dometributes['scale'], translation=dometributes['trans'][dome_name], rx='-90')
        # else:
        #     domeCoverCollis.createObject('MechanicalObject', name='collisMech',  scale=dometributes['scale'], \
        #                                         translation=dometributes['trans'][dome_name], rx=0)
        # domeCoverCollis.createObject('Triangle', selfCollision="false")
        # domeCoverCollis.createObject('Line',selfCollision="false")
        # domeCoverCollis.createObject('Point', selfCollision="false")
        # domeCoverCollis.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')# , input='@../dh_dofs',  output='@DomeCavity', template="Vec3d")


        # COUCH COMPOZ
        CouchFoot, CouchFootCollis = make_couch_compos(rootNode, 'CouchFoot', create_solver=True)
        CouchLatBase, CouchLatBaseCollis = make_couch_compos(CouchFoot, 'CouchLatBase', mesh='{}/truebeam/CouchLatBase.stl'.format(couch['couches_dir']), create_solver=False)
        CouchMain, CouchMainCollis = make_couch_compos(CouchLatBase, 'CouchMain', mesh='{}/truebeam/CouchMain.stl'.format(couch['couches_dir']), create_solver=False)
        CouchLngInner, CouchLngInnerCollis = make_couch_compos(CouchMain, 'CouchLngInner', mesh='{}/truebeam/CouchLngInner.stl'.format(couch['couches_dir']), create_solver=False)
        CouchLngMid, CouchLngMidCollis = make_couch_compos(CouchLngInner, 'CouchLngMid', mesh='{}/truebeam/CouchLngMid.stl'.format(couch['couches_dir']), create_solver=False)
        CouchBlock, CouchBlockCollis = make_couch_compos(CouchLngMid, 'CouchBlock', mesh='{}/couch_block.stl'.format(couch['couches_dir']), create_solver=False)
        CouchTop, CouchTopCollis = make_couch_compos(CouchLngMid, 'CouchTop', mesh='{}/truebeam/CouchTop.stl'.format(couch['couches_dir']), create_solver=False)
        HeadSupport, HeadSupportCollis = make_couch_compos(CouchTop, 'HeadSupport', mesh='{}/headsupport.stl'.format(couch['couches_dir']), create_solver=False)
        CouchPad, CouchPadCollis = make_couch_compos(CouchTop, 'CouchPad', mesh='{}/couchpad.stl'.format(couch['couches_dir']), create_solver=False)

        # Gantry | Not needed for MRI
        if setuptype['LINAC']:
            Gantry, _ =  make_couch_compos(rootNode, 'TBGantry', mesh='{}/truebeam/TBGantry.stl'.format(couch['couches_dir']))
            TurnTable, _ = make_couch_compos(Gantry, 'TurnTable', create_solver=False)
        if setuptype['Flexis']:
            # make_flexis(patient, 'flexi_base', mesh='{}/flexi_base.stl'.format(flexi['flexis_dir']))
            make_flexis(patient, 'flexi_right', mesh='{}/flexi_sides.stl'.format(flexi['flexis_dir']))
            make_flexis(patient, 'flexi_left', mesh='{}/flexi_sides.stl'.format(flexi['flexis_dir']))


        return rootNode;

    def initGraph(self, node):
        'Nothing to do here Scotty.'
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
