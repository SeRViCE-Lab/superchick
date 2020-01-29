#define SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_CPP

#include "IABPlugin/ForceFields/Tetra/include/TetrahedronMooneyRivlinFEMForceField.inl"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

//////////****************To register in the factory******************

// Register in the Factory
int TetrahedronMooneyRivlinFEMForceFieldClass = core::RegisterObject("TetrahedronMooneyRivlinFEMForceField")
.add< TetrahedronMooneyRivlinFEMForceField<Vec3Types> >()
// .add< TetrahedronMooneyRivlinFEMForceField<Vec2Types> >()
// .add< TetrahedronMooneyRivlinFEMForceField<Vec1Types> >()
// .add< TetrahedronMooneyRivlinFEMForceField<Vec6Types> >()
.addAuthor("Lekan Molu")
.addLicense("BSD")
.addAlias("TetraMRFF")
  ;

template class SOFA_IABPlugin_API TetrahedronMooneyRivlinFEMForceField<Vec3Types>;
// template class TetrahedronMooneyRivlinFEMForceField<Vec2Types>;
// template class TetrahedronMooneyRivlinFEMForceField<Vec1Types>;
// template class TetrahedronMooneyRivlinFEMForceField<Vec6Types>;


} // namespace forcefield

} // namespace component

} // namespace sofa
