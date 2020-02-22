
import sys
import Sofa

class patient_rigid_whole (Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments) :
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):

        # rootNode
        rootNode.createObject('CGLinearSolver', threshold='1e-18', tolerance='1e-12', template='GraphScattered', iterations='25')
        rootNode.createObject('MeshObjLoader', name='patient_loader', filename='../../../../ros/srs_traj_opt/patient_description/meshes/patient.obj')
        rootNode.createObject('MechanicalObject', src='@patient_loader', name='patient-states', template='Vec3d')
        rootNode.createObject('UniformMass', totalMass='95')

        # rootNode/PatientVisu
        PatientVisu = rootNode.createChild('PatientVisu')
        self.PatientVisu = PatientVisu
        PatientVisu.createObject('OglModel', src='@../patient_loader', scale='1.0', name='PatientVisual', color='', rx='0', ry='0', rz='0', translation='0 0 0')

        # rootNode/PatientCollis
        PatientCollis = rootNode.createChild('PatientCollis')
        self.PatientCollis = PatientCollis
        PatientCollis.createObject('MeshObjLoader', name='patient-collis-loader', filename='../../../../ros/srs_traj_opt/patient_description/meshes/patient.obj')
        PatientCollis.createObject('MechanicalObject', force='0 0 0', name='patient-collis-states', template='Vec3d', position='0 0 0', velocity='0 0 0', externalForce='0 0 0', restScale='1')

        return 0;

def createScene(rootNode):
    try :
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    mypatient_rigid_whole = patient_rigid_whole(rootNode,commandLineArguments)
    return 0;
