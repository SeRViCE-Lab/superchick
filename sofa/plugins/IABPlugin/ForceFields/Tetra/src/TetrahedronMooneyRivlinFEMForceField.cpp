#define SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_CPP

#include "IABPlugin/ForceFields/Tetra/include/TetrahedronMooneyRivlinFEMForceField.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

//////////****************To register in the factory******************

// Register in the Factory
int TetrahedronMooneyRivlinFEMForceFieldClass = core::RegisterObject("Tetrahedron MooneyRivlin FEM ForceFieldClass")
  .add< TetrahedronMooneyRivlinFEMForceField<Vec3Types> >()

  ;

template class TetrahedronMooneyRivlinFEMForceField<Vec3Types>;


} // namespace forcefield

} // namespace component

} // namespace sofa
