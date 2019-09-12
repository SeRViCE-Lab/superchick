/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_INL

#include "include/debuggers.h"
#include "ForceFields/integrand.h"
#include "ForceFields/IsochoricForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/RGBAColor.h>
#include <sofa/helper/system/config.h>
#include <sofa/helper/rmath.h>
#include <cassert>
#include <iostream>

// see SoftRobots/model/SurfacePressureModel
namespace sofa
{

namespace component
{

namespace forcefield
{
// Constructor of the class IsochoricForceField
// initializing data with their default value (here d_inputForTheUser=20)
template<class DataTypes>
IsochoricForceField<DataTypes>::IsochoricForceField()
    : indices(initData(&indices, "indices", "index of nodes controlled by the isochoric deformations")),
    Ri(initData(&Ri, "Ri", "internal radius in the reference configuration")),
    Ro(initData(&Ro, "Ro", "external radius in the reference configuration")),
    Ro(initData(&ri, "0", "internal radius in the current configuration")),
    Ro(initData(&ro, "0", "external radius in the current configuration")),
    C1(initData(&C1, "0", "material elasticity of the internal IAB wall")),
    C2(initData(&C2, "0", "material elasticity of the outer IAB wall")),
    rho(initData(&rho, "0.0984", "mass density of the system")),
    nu(initData(&nu, "0.45", "Poisson ratio of the IAB materials" )),
    color(initData(&color,sofa::defaulttype::Vec4f(0.0,.7, .8,1.0), "color","color"))
    mode(initData(&mode, "expand", "mode of deformation: <expansion> or <compression>"))
{
  //default Constructor
  init()
}


template<class DataTypes>
IsochoricForceField<DataTypes>::~IsochoricForceField()
{
}


template<class DataTypes>
void IsochoricForceField<DataTypes>::init()
{
    // ripped off angularSpringForceField
    core::behavior::ForceField<DataTypes>::init();

    if((ri==0) || (ro==0))
    {
      std::cout << "Understand that these ri and ro values are bonkers" << std::endl;
      std::terminate();
    }
    // Initialization of your ForceField class and variables
    // perhaps initialize all spheres with a default internal and external radius in reference configuration for now
    // C1 = 1.1e4F;
    // C2 = 2.2e4F;
    abstol = 1e-2F;
    reltol = 1e-5F;

    mState = dynamic_cast<core::behavior::MechanicalState<DataTypes> *> (this->getContext()->getMechanicalState());
    if (!mState) {
		msg_error("IsochoricForceField") << "MechanicalStateFilter has no binding MechanicalState" << "\n";
    }
    matS.resize(mState->getMatrixSize(), mState->getMatrixSize());
}


template<class DataTypes>
void IsochoricForceField<DataTypes>::reinit()
{
  // not yet implemented
}

template<class DataTypes>
void IsochoricForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                             DataVecDeriv& f,
                                             const DataVecCoord& x,
                                             const DataVecDeriv& datav1)
{
    if(!mState) {
    msg_info("IsochoricForceField") << "No Mechanical State found, no force will be computed..." << "\n";
        return;
    }
    // Compute the forces f from the current DOFs p; here i am using the derived stress from eq 25v in paper 1
    helper::WriteAccessor< DataVecDeriv1 > f1 = f;
    helper::ReadAccessor< DataVecCoord1 >  p1 = x;
    helper::ReadAccessor< DataVecDeriv1 >  v1 = datav1;

    f1.resize(p1.size());

    // radii to take IAB into in the current configuration
    double ri = Ri;
    double ro = std::cbrt(std::pow(Ro, 3) - std::pow(Ri, 3)) + std::pow(ri, 3);

    // calculate the stress and pressure needed to go from a reference configuartion to a current configuration
    double radial_stress_n = integrator<float>(Ri, Ro, abstol, reltol,
                                                C1, C2, radial_stress_r2c<float>\
                                                (ri=ri, ro=ro,
                                                  Ri=Ri, C1, C2));
    double internal_pressure = integrator<float>(Ri, Ro, abstol, reltol,
                                                C1, C2, radial_stress_r2c<float>(ri=ri, \
                                                  ro=ro,
                                                  Ri=Ri, // infer Ro from Ri
                                                  C1, C2));
    OUT_INFO("Calculated pressure based on given parameters:\n\t [C1, C2, Ri, Ro, ri, ro]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] is \n\t P=%.4f N/m^2" <<
              C1, C2, Ri, Ro, ri, ro, internal_pressure);
    printf("Calculated normal stress based on given parameters:\n\t [C1, C2, Ri, Ro, ri, ro]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] is \n\t Sigma=%.4f " <<
              C1, C2, Ri, Ro, ri, ro, radial_stress_n);
}


template<class DataTypes>
void IsochoricForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams,
                                              DataVecDeriv& d_df , const DataVecDeriv& d_dx)
{
    // Compute the force derivative d_df from the current, which will be multiplied with the field d_dx
}


template<class DataTypes>
void IsochoricForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix * /* mat */,
                                                 SReal /* k */, unsigned int & /* offset */)
{
    // Compute the force derivative d_df from the current and store the resulting matrix
}


template<class DataTypes>
void IsochoricForceField<DataTypes>::addKToMatrix(const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/,
                                                 SReal /*kFact*/)
{
    // Same as previously
    // but using accessor
}


template <class DataTypes>
SReal IsochoricForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams* /*params*/,
                                                        const DataVecCoord& x) const
{
    // Compute the potential energy associated to the force f
}




} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_INL
