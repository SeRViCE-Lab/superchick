### Using Sphere as a TEST

These are the components that are set in sphere_node:

```
[WARNING] [DAGNode(SphereNode)] BaseAnimationLoop: defaultAnimationLoop
[WARNING] [DAGNode(SphereNode)] OdeSolver: eulerImplicitSolver1
[WARNING] [DAGNode(SphereNode)] LinearSolver: cGLinearSolver1
[WARNING] [DAGNode(SphereNode)] ConstraintSolver: VisualLoop: defaultVisualManagerLoop
[WARNING] [DAGNode(SphereNode)] InteractionForceField: Springs
[WARNING] [DAGNode(SphereNode)] ForceField: matrixMass1 isochoricFF1
[WARNING] [DAGNode(SphereNode)] State: spherical-states
[WARNING] [DAGNode(SphereNode)] MechanicalState: spherical-states
[WARNING] [DAGNode(SphereNode)] Mechanical Mapping:
[WARNING] [DAGNode(SphereNode)] Mapping:
[WARNING] [DAGNode(SphereNode)] Topology:
[WARNING] [DAGNode(SphereNode)] MeshTopology:
[WARNING] [DAGNode(SphereNode)] Shader:
[WARNING] [DAGNode(SphereNode)] ProjectiveConstraintSet:
[WARNING] [DAGNode(SphereNode)] ConstraintSet:
[WARNING] [DAGNode(SphereNode)] BehaviorModel:
[WARNING] [DAGNode(SphereNode)] VisualModel: defaultVisualManagerLoop visualStyle
[WARNING] [DAGNode(SphereNode)] CollisionModel:
[WARNING] [DAGNode(SphereNode)] ContextObject:
[WARNING] [DAGNode(SphereNode)] Pipeline:
```

#### MechanicalComputeEnergyVisitor
For the energy of the system's mechanical degrees of freedom, see class MechanicalComputeEnergyVisitor in v19.06/SofaKernel/framework/sofa/simulation/MechanicalComputeEnergyVisitor.cpp

- This processes the fwdMass (kinetic energy) and fwdForceField (potential energy)

#### MechanicalGetMomentumVisitor
See v19.06/SofaKernel/framework/sofa/simulation/MechanicalGetMomentumVisitor

- This returns the 6 vector momenta of the system. In a fwdMass method, the momenta of the mass can be found

#### MechanicalMatrixVisitor

See v19.06/SofaKernel/framework/sofa/simulation/MechanicalMatrixVisitor
- This computes the size of a mass or stiffness of the whole scene as a mechanical matrix

#### Create a xml scene in c++
See sofa/v19.06/SofaKernel/modules/sofa/frameworkextra/frameworkextra_test/simulation/Node_test.cpp#L95

#### Might be useful to check events in

+ `sofa/v19.06/SofaKernel/framework/sofa/core/objectmodel/GUIEvent.cpp`

+ `sofa/v19.06/SofaKernel/framework/sofa/core/objectmodel/HapticDeviceEvent.h`

+ `sofa/v19.06/SofaKernel/framework/sofa/core/objectmodel/DetachNodeEvent.cpp`

#### MechanicalState API is defined in

+ `sofa/v19.06/SofaKernel/framework/sofa/core/behavior/MechanicalState.h`

+ `sofa/v19.06/SofaKernel/framework/sofa/core/behavior/MechanicalState.cpp`

##### Transform a state

 + We can `applyTranslation` or `applyRotation`  or `applyScale` or `addBox`
     - See core/behavior/BaseMechanicalState

 + We can also `printDOF`, `printDOFWithElapsedTime`, `initGnuplot`, `exportGnuplot`
     - see `/sofa/v19.06/SofaKernel/framework/sofa/core/behavior/BaseMechanicalState.cpp`


#### Run Debug Info      
Currently, simulating a single IAB returns no mechanical state in simulation
`[INFO]    [IsochoricForceField] No Mechanical State found, no force will be computed...
`

+ computation of `ro` is correct. Pressure seems way off from matlab code.
