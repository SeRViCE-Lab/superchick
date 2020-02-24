__author__ = "Lekan Ogunmolu"
__description__ = "Lekan generated this file on 02/13/2020"
__examples__ = "See SoftRobotsPlugin Examples  \
                (i)  /sofa/applications/plugins/SoftRobots/docs/\
                examples/component/constraint/SurfacePressureConstraint/SurfacePressureConstraint.pyscn \
                (ii) python/pneunets.py"

import Sofa
from utils import *

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
        self.patient = rootNode.createChild('patient')
        self.patient.createObject('EulerImplicit', name='odesolver')
        self.patient.createObject('SparseLDLSolver', name='linearSolver')
        self.patient.createObject('MeshObjLoader', name='patient_loader', filename='{}/patient.obj'.format(dometributes['meshes_dir']))
        self.patient.createObject('MechanicalObject', src='@patient_loader', name='patient-states', template='Vec3d', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])
        # scale="250", translation=trans['neck_left'], rotation=rot['neck_left'])
        self.patient.createObject('UniformMass', totalMass='95')
        self.patient.createObject('UncoupledConstraintCorrection')
        patientCollis = self.patient.createChild('patientCollis')
        patientCollis.createObject('Mesh', src="@../patient_loader")
        patientCollis.createObject('MechanicalObject', name='patient_dofs', template='Vec3d', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])
        patientVisu = self.patient.createChild('patientVisu')
        patientVisu.createObject('OglModel', src='@../patient_loader', name='patientVisual', scale=patributes['scale'], \
                                    rx=patributes['rx'], ry=patributes['ry'], translation=patributes['translation'])

        self.TurnTable, TurnTableCollis = make_couch_compos(rootNode, 'TurnTable')
        self.CouchFoot, CouchFootCollis = make_couch_compos(rootNode, 'CouchFoot')
        self.CouchLatBase, CouchLatBaseCollis = make_couch_compos(rootNode, 'CouchLatBase', mesh='{}/truebeam/CouchLatBase.stl'.format(couch['couches_dir']))
        self.CouchMain, CouchMainCollis = make_couch_compos(rootNode, 'CouchMain', mesh='{}/truebeam/CouchMain.stl'.format(couch['couches_dir']))
        self.CouchLngInner, CouchLngInnerCollis = make_couch_compos(rootNode, 'CouchLngInner', mesh='{}/truebeam/CouchLngInner.stl'.format(couch['couches_dir']))
        self.CouchLngMid, CouchLngMidCollis = make_couch_compos(rootNode, 'CouchLngMid', mesh='{}/truebeam/CouchLngMid.stl'.format(couch['couches_dir']))
        self.CouchTop, CouchTopCollis = make_couch_compos(rootNode, 'CouchTop', mesh='{}/truebeam/CouchTop.stl'.format(couch['couches_dir']))
        self.HeadSupport, HeadSupportCollis = make_couch_compos(rootNode, 'HeadSupport', mesh='{}/headsupport.stl'.format(couch['couches_dir']))
        self.CouchPad, CouchPadCollis = make_couch_compos(rootNode, 'CouchPad', mesh='{}/couchpad.stl'.format(couch['couches_dir']))
        self.CouchBlock, CouchBlockCollis = make_couch_compos(rootNode, 'CouchBlock', mesh='{}/couch_block.stl'.format(couch['couches_dir']))

        self.base_neck_left = dome_maker(rootNode, 'base_neck_left')
        self.base_neck_right = dome_maker(rootNode, 'base_neck_right')
        self.base_skull_left = dome_maker(rootNode, 'base_skull_left')
        self.base_skull_right = dome_maker(rootNode, 'base_skull_right')

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
