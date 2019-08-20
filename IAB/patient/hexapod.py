# -*- coding: utf-8 -*-
"""
Step 1:
We are introducing basic mechanical modeling, the new components bring
time integration and a mechanical object to the scene .
"""
from stlib.scene import Scene

# below from setup.pyscn
from stlib.scene import MainHeader, ContactHeader
# A prefab that implements an ElasticMaterialObject
from stlib.physics.deformable import ElasticMaterialObject
from stlib.visuals import ShowGrid
from stlib.physics.rigid import Floor
from stlib.physics.rigid import Sphere


# This function includes the whole mechanical model of the silicone piece, as written in the previous step, except that the prefab ElasticMaterialObject is used, instead of creating each component.
def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    # Prefab ElasticMaterialObject implementing the whole mechanical model of the silicone piece
    # e = ElasticMaterialObject(body,
    #                           volumeMeshFileName="../compoz/table_stand.stl",
    #                           poissonRatio=0.45,
    #                           youngModulus=800,
    #                           totalMass=0.032,
    #                           rotation=[90, 0, 0])

    # Visual model
    # visual = e.createChild("Visual")
    visual = parent.createChild("Visual")
    visual.createObject("MeshSTLLoader",
                        name="loader",
                        filename="../compoz/table_stand.stl",
                        rotation=[90, 0, 0])
    visual.createObject("OglModel",
                        name="renderer",
                        src="@loader",
                        color=[1.0, 1.0, 1.0, 0.5])
    visual.createObject("BarycentricMapping",
                        input=e.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())
    return body

# def createScene(rootNode):
#     rootNode.createObject("RequiredPlugin", name="SofaSparseSolver")

#     # Setting the gravity, assuming the length unit is in millimeters
#     scene = Scene(rootNode, gravity=[0.0, -9810, 0.0])

#     # Setting the timestep in seconds
#     rootNode.dt = 0.001

#     # Graphic modelling of the legends associated to the servomotors
#     scene.createObject("MeshSTLLoader", name="loader1", filename="../compoz/table_stand.stl")
#     scene.createObject("OglModel", src="@loader1")

#     # # Tool to load the mesh file of the silicone piece. It will be used for both the mechanical and the visual models.
#     # rootNode.createObject("MeshSTLLoader", name="loader2", filename="../compoz/table-slab.stl")

#     # Basic mechanical modelling of the silicone piece
#     # elasticbody = rootNode.createChild("MechanicalBody")
#     # elasticbody.createObject("MechanicalObject", name="dofs",
#     #                          position=rootNode.loader2.position,
#     #                          showObject=True, showObjectScale=5.0,
#     #                          rotation=[90.0, 0.0, 0.0])
#     # elasticbody.createObject("UniformMass")

#     # elasticbody.createObject("EulerImplicitSolver")
#     # elasticbody.createObject("SparseLDLSolver")

#     # # Visual object
#     # visual = rootNode.createChild("Visual")
#     # # The mesh used for the Visual object is the same as the one for the MechanicalObject, and has been introduced in the rootNode
#     # visual.createObject("OglModel", name="renderer",
#     #                     src='@../loader2',
#     #                     color=[1.0, 1.0, 1.0, 0.5])

#     # A mapping applies the deformations computed on the mechanical model (the input parameter)
#     # to the visual model (the output parameter).
#     elasticbody.createObject("IdentityMapping",
#                              input=elasticbody.dofs.getLinkPath(),
#                              output=visual.renderer.getLinkPath())

def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0])
    rootNode.dt = 0.025
    scene.VisualStyle.displayFlags = "showBehavior"

    scene.createObject("MeshSTLLoader", name="loader", filename="../compoz/table_stand.stl")
    scene.createObject("OglModel", src="@loader")

    # Instanciating the prefab into the graph
    body = ElasticBody(rootNode)

    # Instanciating the FixingBox prefab into the graph, constraining the mechanical object of the ElasticBody.
    fix = FixingBox(rootNode,
                    body.ElasticMaterialObject,
                    translation=[0.0, 0.0, 0.0],
                    scale=[30., 30., 30.])

    # Changing the property of the Box ROI so that the constraint area appears on screen.
    fix.boxroi.drawBoxes = True
