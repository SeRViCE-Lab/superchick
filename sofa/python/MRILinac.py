__author__ = "Lekan Ogunmolu"
__description__ = "Lekan generated this file on 02/13/2020"
__examples__ = "See SoftRobotsPlugin Examples  \
                (i)  /sofa/applications/plugins/SoftRobots/docs/\
                examples/component/constraint/SurfacePressureConstraint/SurfacePressureConstraint.pyscn \
                (ii) python/pneunets.py"

import matplotlib as mpl
mpl.use('qt5agg')
import Sofa
import logging
from utils import *
from config import *
from kinecontrol import controller

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
logger = logging.getLogger(__name__)

class dome_test (Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments) :
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        if setuptype['Controller'] == 'open_loop':
            self.controller = "kinecontrol/ol_control.py"
        elif setuptype['Controller'] == 'diff_kine':
            self.controller = "kinecontrol/diff_kine_controller.py"
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):
        rootNode.createObject('APIVersion', level='19.06')
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

        # this from here:https://www.sofa-framework.org/community/forum/topic/rigid-objects-passing-through-deformable/
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')

        rootNode.createObject('FreeMotionAnimationLoop')
        rootNode.createObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')
        rootNode.createObject('PythonScriptController', filename=self.controller, classname="controller")

        # Patient
        patient = rootNode.createChild('patient')
        patient.createObject('EulerImplicit', name='patientOdeSolver')
        # patient.createObject('SparseLDLSolver', name='patient_preconditioner')
        patient.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')
        # patient.createObject('MeshObjLoader', name='patient_loader', filename='{}/patient.obj'.format(dometributes['meshes_dir']))
        patient.createObject('MeshObjLoader', name='patient_loader', filename='{}/patient_remeshed2k.obj'.format(dometributes['meshes_dir']))
        patient.createObject('MechanicalObject', src='@patient_loader', name='patient_dofs', template='Vec3d', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])
        patient.createObject('UniformMass', totalMass='95')
        patient.createObject('UncoupledConstraintCorrection')
        patientCollis = patient.createChild('patientCollis')
        patientCollis.createObject('Mesh', src="@../patient_loader")
        patientCollis.createObject('MechanicalObject', name='patient_collis_dofs', template='Vec3d', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])
        patient.createObject('BoxROI', name='boxROI', box='-1.2 1.2 -0.5 1.5 -0.48166 0.811406', drawBoxes='true', doUpdate='1')#, position="@dh_dofs.rest_position", tetrahedra="@TetraTopologyContainer.tetrahedra")
        # this defines the boundary condition which creates springs between the current position of the body and its initial position
        patient.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e1', angularStiffness='1e1') # stiffness was 1e12 after the pneunets demo

        patientVisu = patient.createChild('patientVisu')
        patientVisu.createObject('OglModel', src='@../patient_loader', name='patientVisual', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])
        # create pointer towards the MechanicalObject
        # patientObjectPointer = patient.getObject('patient_dofs')

        # IABs
        base_neck_left = make_base_domes(rootNode, 'base_neck_left')
        base_neck_right = make_base_domes(rootNode, 'base_neck_right')
        base_skull_left = make_base_domes(rootNode, 'base_skull_left')
        base_skull_right = make_base_domes(rootNode, 'base_skull_right')

        # make side_domes
        side_fore_left = make_side_domes(rootNode, 'side_fore_left')
        side_chin_left = make_side_domes(rootNode, 'side_chin_left')
        # side right domes
        side_fore_right = make_side_domes(rootNode, 'side_fore_right', right_domes=True)
        side_chin_right = make_side_domes(rootNode, 'side_chin_right', right_domes=True)

        # revisit after deadline
        # side_fore_left = make_domes(rootNode, 'side_fore_left', side_domes=True)
        # side_chin_left = make_domes(rootNode, 'side_chin_left', side_domes=True)
        # # side right domes
        # side_fore_right = make_domes(rootNode, 'side_fore_right', right_domes=True)
        # side_chin_right = make_domes(rootNode, 'side_chin_right', right_domes=True)

        # Gantry | Not needed for MRI
        if setuptype['LINAC']:
            Gantry, _ =  make_couch_compos(rootNode, 'TBGantry', mesh='{}/truebeam/TBGantry.stl'.format(couch['couches_dir']))
            TurnTable, _ = make_couch_compos(Gantry, 'TurnTable', create_solver=False)
        if setuptype['Flexis']:
            # make_flexis(patient, 'flexi_base', mesh='{}/flexi_base.stl'.format(flexi['flexis_dir']))
            make_flexis(patient, 'flexi_right', mesh='{}/flexi_sides.stl'.format(flexi['flexis_dir']))
            make_flexis(patient, 'flexi_left', mesh='{}/flexi_sides.stl'.format(flexi['flexis_dir']))

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
