import os
from os.path import join

'''Convention:
    Bottom IABs: {{neck_left, neck_right},{skull_left, skull_right}}
    Side   IABs: {{fore_left, chin_left}, {fore_right, chin_right}}
'''


# patient attributes
patributes = dict(scale="250", rx="-90", ry="-90", translation='-300 80 0')
# couch components' attributes
couch = dict(CouchFoot=dict(scale='5e-6*.91', color="0.3 0.6 0.7", mass='50', trans='500 50 20'),
             TurnTable=dict(scale='5e-6*.91', color="0.8549 0.8235 0.6431", mass='30', trans='500 50 20'),
             CouchLatBase=dict(scale='5e-6*.91', color="magenta", mass='40', trans='500 50 20'),
             CouchMain=dict(scale='5e-6*.91', color="cyan", mass='35', trans='500 50 20'),
             CouchLngInner=dict(scale='5e-6*.91', color="cyan", mass='15', trans='500 50 20'),
             CouchLngMid=dict(scale='5e-6*.91', color="cyan", mass='12', trans='500 50 20'),
             CouchTop=dict(scale='5e-6*.91', color="cyan", mass='25', trans='500 50 20'),
             HeadSupport=dict(scale='5e-6*.91', color="cyan", mass='25', trans='0 90 0'),
             CouchPad=dict(scale='5e-6*1', color="0.3 0.6 0.7", mass='30', trans='500 50 20'),
             CouchBlock=dict(scale='100', color="yellow", mass='100', trans='500 50 20'),
             couches_dir = join(os.getcwd(), '../..', 'ros/srs_traj_opt/couch_description/meshes'),
             )
# individual dome arretibues
dometributes = dict(scale=25, mass=.04, poisson=.3,youngs=18000,
                    youngsmod=500,youngsmodstiff=1500,
                    pressureConsVal="0.0001",
                    trans=dict(base_neck_left='-100 -10 60', # base iabs
                                base_neck_right='-100 -10 -90',
                                base_skull_left='50 -10 60',
                                base_skull_right='50 -10 -90',
                                side_fore_left='20 50 80',  # side iabs
                                side_fore_right='20 50 -180',
                                side_chin_left='100 40 80',
                                side_chin_right='-100 40 -180',
                                side_patient='-300 25 -50'),
                    # rotation vectors
                    rot=dict(base_neck_left='-90 0 0', # base iabs
                            base_neck_right='-90 0 0',
                            base_skull_left='-90 0 0',
                            base_skull_right='-90 0 0',
                            side_fore_left='180 0 0',  # side iabs
                            side_fore_right='-180 0 0',
                            side_chin_left='-180 0 0',
                            side_chin_right='0 0 0',
                            patient='-90 -90 0'),
                    meshes_dir = join(os.getcwd(), '../..', 'ros/srs_traj_opt/patient_description/meshes'),
                    pod_dir = join(os.getcwd(), '../..', 'ros/srs_traj_opt/hexapod_description/meshes')

                    ) # as measured by newly fabricated soft robot, in Kg

def make_couch_compos(rootNode, itemname, mesh=''):
    'this function solves for the '
    node = rootNode.createChild(itemname)
    node.createObject('EulerImplicit', name='{}_odesolver'.format(itemname))
    node.createObject('SparseLDLSolver', name='{}_linearSolver'.format(itemname))
    if mesh:
        node.createObject('MeshSTLLoader', name='{}_loader'.format(itemname), filename='{}'.format(mesh))
    else:
        node.createObject('MeshSTLLoader', name='{}_loader'.format(itemname), filename='{}/truebeam/Assembly4/{}.stl'.format(couch['couches_dir'], itemname))
    node.createObject('Mesh', src='@{}_loader'.format(itemname), name='{}_mesh'.format(itemname))
    node.createObject('MechanicalObject', src='@{}_loader'.format(itemname), name="{}_dofs".format(itemname), \
         template='Vec3d', scale=couch['{}'.format(itemname)]['scale'], translation=couch['{}'.format(itemname)]['trans'])
    node.createObject('UniformMass', totalMass=couch['{}'.format(itemname)]['mass'])
    node.createObject('OglModel', color=couch[itemname]['color'], src='@{}_loader'.format(itemname), name='{}Visu'.format(itemname), \
         template='Vec3d', scale=couch['{}'.format(itemname)]['scale'], translation=couch['{}'.format(itemname)]['trans'])
    node_collis = node.createChild('{}Collis'.format(itemname))
    node_collis.createObject('MeshTopology', name='meshTopology')
    node_collis.createObject('MechanicalObject', src='@../{}_loader'.format(itemname), name='{}_collis_dofs'.format(itemname), template='Vec3d')

    return node, node_collis


def dome_maker(rootNode, dome_name):
    # neck left
    dome=rootNode.createChild(dome_name)
    dome.createObject('EulerImplicitSolver', name='{}_odesolver'.format(dome_name), printLog='false')
    dome.createObject('SparseLDLSolver', name='{}_linearSolver'.format(dome_name))
    dome.createObject('MeshVTKLoader', name='{}_loader'.format(dome_name), filename='{}/dome/dome.vtu'.format(dometributes['meshes_dir']),rotation=dometributes['rot'][dome_name])#'_90 0 0')
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
    # rootNode.createObject('TetrahedronMooneyRivlinFEMForceField', name='rootFEM', materialName='MooneyRivlinIncompressible', ParameterSet='1000 100', template='Vec3d', poissonRatio='0.45', youngModulus='10000')
    ##########################################
    #             Sub topology               #
    ##########################################

    domeSubTopo = dome.createChild('DomeHeadSubTopo')
    domeSubTopo.createObject('TetrahedronSetTopologyContainer', position='@../{}_loader.position'.format(dome_name), tetrahedra="@boxROISubTopo.tetrahedraInROI", name='container')
    domeSubTopo.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=dometributes['poisson'],
                                        youngModulus=str(dometributes['youngsmodstiff'] - dometributes['youngsmod']))
    # rootNode/DomeCavity
    domeCavity = dome.createChild('DomeCavity')
    domeCavity.createObject('MeshObjLoader', name='cavityLoader', filename='{}/dome/dome_cavity.obj'.format(dometributes['meshes_dir']), triangulate="true")
    domeCavity.createObject('Mesh', src='@cavityLoader', name='cavityTopo')
    domeCavity.createObject('MechanicalObject',  name='domeCavity', scale=dometributes['scale'], translation=dometributes['trans'][dome_name], rx=dometributes['rot'][dome_name][0])
    domeCavity.createObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template="Vec3d", value=dometributes['pressureConsVal'], triangles='@cavityTopo.triangles', drawPressure='0', drawScale='0.0002', valueType='pressure') #
    domeCavity.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')
    # rootNode/DomeHeadCollis
    domeCollis = dome.createChild('DomeHeadCollis')
    domeCollis.createObject('MeshObjLoader', name='domeCollisLoader', filename='{}/dome/dome_cavity.obj'.format(dometributes['meshes_dir'])) #src='@../domeVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
    domeCollis.createObject('Mesh', src='@domeCollisLoader', name='topo')
    domeCollis.createObject('MechanicalObject', name='collisMech',  scale=dometributes['scale'], translation=dometributes['trans'][dome_name], rx=dometributes['rot'][dome_name][0])
    domeCollis.createObject('Triangle', selfCollision="false")
    domeCollis.createObject('Line',selfCollision="false")
    domeCollis.createObject('Point', selfCollision="false")
    domeCollis.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')# , input='@../dh_dofs',  output='@DomeCavity', template="Vec3d")
    # rootNode/DomeHeadVisu
    domeVisu = dome.createChild('DomeHeadVisu')
    domeVisu.createObject('MeshObjLoader', name='domeVisuLoader', filename='{}/dome/dome.obj'.format(dometributes['meshes_dir'])) #src='@../domeVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
    domeVisu.createObject('OglModel',  color='0.3 0.2 0.2 0.6') # name='@domeVisuLoader', template='Vec3d', src="@../domeCollisLoader", #dx="20", dy="-80", dz="10", rx="-90",
    domeVisu.createObject('BarycentricMapping', name='mapping')#, mapForces='false', mapMasses='false')
    # Dome Cover
    domeCoverVisu = dome.createChild('DomeCoverVisu')
    domeCoverVisu.createObject('MeshSTLLoader', name='domeCoverLoader', filename='{}/dome/cover.stl'.format(dometributes['meshes_dir']))
    domeCoverVisu.createObject('MechanicalObject', name='domeCoverVisu',  scale=dometributes['scale'], \
                                        translation=dometributes['trans'][dome_name], rx=dometributes['rot'][dome_name][0])
    domeCoverVisu.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')
    domeCoverVisu.createObject('OglModel', color='0.3 0.5 0.5 0.6')
    # DomeCoverCollis
    domeCoverCollis = dome.createChild('DomeCoverCollis')
    domeCoverCollis.createObject('MeshObjLoader', name='domeCoverCollis', filename='{}/dome/cover.obj'.format(dometributes['meshes_dir'])) #src='@../domeHeadVTKLoader', scale='1', , rx='0', ry='0', rz='0', dz='0', dx='0', dy='0', template='Vec3d')
    domeCoverCollis.createObject('Mesh', src='@domeCoverCollis', name='topo')
    domeCoverCollis.createObject('MechanicalObject', name='collisMech',  scale=dometributes['scale'], \
                                        translation=dometributes['trans'][dome_name], rx=dometributes['rot'][dome_name][0] )
    domeCoverCollis.createObject('Triangle', selfCollision="false")
    domeCoverCollis.createObject('Line',selfCollision="false")
    domeCoverCollis.createObject('Point', selfCollision="false")
    domeCoverCollis.createObject('BarycentricMapping',  name='mapping', mapForces='false', mapMasses='false')# , input='@../dh_dofs',  output='@DomeCavity', template="Vec3d")
