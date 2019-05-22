from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from stlib.physics.collision import CollisionMesh
from splib.objectmodel import SofaPrefab, SofaObject
from stlib.scene import Scene


def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="data/mesh/tripod_mid.gidmsh",
                              # volumeMeshFileName="../compoz/table-slab.gid/table-slab.msh",
                              translation=[0.0, 30, 0.0], rotation=[90, 0, 0],
                              youngModulus=800, poissonRatio=0.45, totalMass=0.032)

    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename="../compoz/table_stand.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                        input=e.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    CollisionMesh(e, surfaceMeshFileName="../compoz/table-slab.stl", name="silicone", translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

    return body


@SofaPrefab
class Quadropod(SofaObject):

    def __init__(self, parent, name="Quadropod", radius=60, numMotors=4, angleShift=180.0):
        self.node = parent.createChild(name)
        body = ElasticBody(self.node)

        dist = radius
        self.numMotors = numMotors
        self.actuatedarms = []
        for i in range(0, self.numMotors):
            name = "ActuatedArm_{}".format(i)
            fi = float(i)
            fnumstep = float(self.numMotors)
            angle = fi*360/fnumstep
            angle2 = fi*360/fnumstep+angleShift
            eulerRotation = [0, angle, 0]
            translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

            self.actuatedarms.append(ActuatedArm(self.node, name=name,
                                                 translation=translation, eulerRotation=eulerRotation,
                                                 attachingTo=body.ElasticMaterialObject))
            self.actuatedarms[i].servomotor.angleLimits = [-2.0225, -0.0255]

    def addCollision(self):

        # numstep = numMotors
        for i in range(0, self.numMotors):
            CollisionMesh(self.actuatedarms[i].ServoMotor.ServoBody,
                          surfaceMeshFileName="../compoz/table-slab.stl",
                          name="TopServoCollision", mappingType='RigidMapping')


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0])
    rootNode.dt = 0.025
    scene.VisualStyle.displayFlags = "showBehavior"

    # scene.createObject("MeshSTLLoader", name="loader", filename="data/mesh/blueprint.stl")
    # scene.createObject("OglModel", src="@loader")

    quadropod = Quadropod(rootNode, numMotors=4, radius=90)
    for arm in quadropod.actuatedarms:
        arm.Constraint.BoxROI.drawBoxes = True
