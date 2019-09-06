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

#include "ForceFields/integrand.h"
#include "ForceFields/IsochoricForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/RGBAColor.h>
#include <sofa/helper/system/config.h>
#include <sofa/helper/rmath.h>
#include <cassert>
#include <iostream>

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
    : d_inputForTheUser(initData(&d_inputForTheUser, (int) 20, "inputForTheUser", "Description of the data inputForTheUser")),
    // d_inputForTheUser(initData(&d_inputForTheUser, (float) 0.1, "Ri", "Internal IAB Radius in Reference Configuration"))
    Ri(Ri), Ro(Ro), C1(C1), C2(C2), rho(rho), nu(nu), mode(mode)

{
  //default Constructor
}


template<class DataTypes>
IsochoricForceField<DataTypes>::~IsochoricForceField()
{
}


template<class DataTypes>
void IsochoricForceField<DataTypes>::init()
{
    // Initialization of your ForceField class and variables
    // perhaps initialize all spheres with a default internal and external radius in reference configuration for now

}


template<class DataTypes>
void IsochoricForceField<DataTypes>::reinit()
{
  // not yet implemented
}

template<class DataTypes>
void IsochoricForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                             DataVecDeriv& vecDer,
                                             const DataVecCoord& p,
                                             const DataVecDeriv& datav1)
{
    // Compute the forces f from the current DOFs p; here i am using the derived stress from eq 25v in paper 1
    helper::WriteAccessor< DataVecDeriv1 > f = vecDer;

    helper::ReadAccessor< DataVecCoord1 >  p1 = p;
    helper::ReadAccessor< DataVecDeriv1 >  v1 = datav1;

    // calculate ro
    double inner_ro = std::pow(Ro, 3) + std::pow(ri, 3) - std::pow(Ri, 3);
    double ro = std::pow(inner_ro, (1/3));
    // evaluate the definite stress integrap in eq 25 of continuum1
    float radial_stress_t1, radial_stress_t2, radial_stress, R;
    // evaluate the definite integratral in (25) from Ri to R0
    radial_stress_t1 = 2 * C1 *((1/ro) - R^6/ro^7) - 2* C2(R^6/ro^7 - r0/R^2)
    integral(cos, 0, M_PI / 2, 10);
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
