/******************************************************************************
*
* Adapted from SoftRobots Plugin
*                                                                             *
******************************************************************************/

#include "IABPlugin/constraint/SurfacePressureConstraint.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

namespace _surfacepressureconstraint_
{

/////////////////////////////////// SurfacePressureConstant ///////////////////////////////////////
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

SurfacePressureConstraintResolution::SurfacePressureConstraintResolution(const double& imposedPressure, const double &minVolumeGrowth, const double &maxVolumeGrowth)
    : ConstraintResolution(1)
    , m_imposedPressure(imposedPressure)
    , m_minVolumeGrowth(minVolumeGrowth)
    , m_maxVolumeGrowth(maxVolumeGrowth)
{
}

void SurfacePressureConstraintResolution::init(int line, double**w, double*force)
{
    SOFA_UNUSED(force);

    m_wActuatorActuator = w[line][line];
}

void SurfacePressureConstraintResolution::resolution(int line, double** w, double* d, double* force, double* dfree)
{
    SOFA_UNUSED(w);
    SOFA_UNUSED(d);
    SOFA_UNUSED(dfree);


    double volumeGrowth = m_wActuatorActuator*m_imposedPressure + d[line];

    if(volumeGrowth<m_minVolumeGrowth)
    {
        volumeGrowth = m_minVolumeGrowth;
        force[line] -= (d[line]-volumeGrowth) / m_wActuatorActuator ;
    }
    if(volumeGrowth>m_maxVolumeGrowth)
    {
        volumeGrowth = m_maxVolumeGrowth;
        force[line] -= (d[line]-volumeGrowth) / m_wActuatorActuator ;
    }
    else
        force[line] = m_imposedPressure ;


}

/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////// VolumeGrowthConstraintResolution ///////////////////////////////////////

VolumeGrowthConstraintResolution::VolumeGrowthConstraintResolution(const double &imposedVolumeGrowth, const double &minPressure, const double &maxPressure)
    : ConstraintResolution(1)
    , m_imposedVolumeGrowth(imposedVolumeGrowth)
    , m_minPressure(minPressure)
    , m_maxPressure(maxPressure)
{
}

void VolumeGrowthConstraintResolution::init(int line, double**w, double*force)
{
    SOFA_UNUSED(force);

    m_wActuatorActuator = w[line][line];
}

void VolumeGrowthConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(w);

    // da=Waa*(lambda_a) + Sum Wai * lambda_i  = m_imposedVolumeGrowth
    lambda[line] -= (d[line]-m_imposedVolumeGrowth) / m_wActuatorActuator ;

    if(lambda[line]<m_minPressure)
        lambda[line] = m_minPressure;
    if(lambda[line]>m_maxPressure)
        lambda[line] = m_maxPressure;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////// FACTORY //////////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int SurfacePressureConstraintClass = core::RegisterObject("This component constrains a model by applying "
                                                          "pressure on surfaces (for example cavities)")
.add< SurfacePressureConstraint<Vec3Types> >(true)//Set Vec3d to default template

;
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_IABPlugin_API SurfacePressureConstraint<Vec3Types>;


}

} // namespace constraintset

} // namespace component

} // namespace sofa
