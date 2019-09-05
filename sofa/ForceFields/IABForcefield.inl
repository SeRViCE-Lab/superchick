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
#ifndef SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_INL

#include <SofaBoundaryCondition/TemplateForceField.h>
#include <sofa/helper/system/config.h>

namespace sofa
{

namespace component
{

namespace forcefield
{


// Constructor of the class TemplateForceField
// initializing data with their default value (here d_inputForTheUser=20)
template<class DataTypes>
TemplateForceField<DataTypes>::TemplateForceField()
    : d_inputForTheUser(initData(&d_inputForTheUser, (int) 20, "inputForTheUser", "Description of the data inputForTheUser"))
{
}


template<class DataTypes>
TemplateForceField<DataTypes>::~TemplateForceField()
{
}


template<class DataTypes>
void TemplateForceField<DataTypes>::init()
{
    // Initialization of your ForceField class and variables
}


template<class DataTypes>
void TemplateForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                             DataVecDeriv& f, const DataVecCoord& p, const DataVecDeriv&)
{
    // Compute the forces f from the current DOFs p
}


template<class DataTypes>
void TemplateForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams,
                                              DataVecDeriv& d_df , const DataVecDeriv& d_dx)
{
    // Compute the force derivative d_df from the current, which will be multiplied with the field d_dx
}


template<class DataTypes>
void TemplateForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix * /* mat */,
                                                 SReal /* k */, unsigned int & /* offset */)
{
    // Compute the force derivative d_df from the current and store the resulting matrix
}


template<class DataTypes>
void TemplateForceField<DataTypes>::addKToMatrix(const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/,
                                                 SReal /*kFact*/)
{
    // Same as previously
    // but using accessor
}


template <class DataTypes>
SReal TemplateForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams* /*params*/,
                                                        const DataVecCoord& x) const
{
    // Compute the potential energy associated to the force f
}




} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_INL
