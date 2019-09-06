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
#ifndef SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_H

#include "ForceFields/config.h"
#include <sofa/core/behavior/ForceField.h>

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/MechanicalParams.h>

#include <sofa/defaulttype/RGBAColor.h>

namespace sofa
{

namespace component
{

namespace isochoricforcefield
{

/// Apply constant forces to given degrees of freedom.
template<class DataTypes>
class IsochoricForceField : public core::behavior::ForceField<DataTypes>
{

public:

    SOFA_CLASS(SOFA_TEMPLATE(IsochoricForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    /// Declare here the data and their type, you want the user to have access to
    Data< float > Ri, Ro; // referencce configuration radius
    Data< float > C1, C2; // material elasticity of iab walls
    Data< float > rho, nu; // iab material densities as well as Poisson ratio of elastic wall
    Data< std::string > mode; // mode tells whether we are expanding or compressing the IABs; accepts "compress" or "expand"
    Data<defaulttype::RGBAColor> color; ///< isochoric spherical forcefield color. (default=[0.0,0.5,1.0,1.0])

    enum { N=DataTypes::spatial_dimensions };
    using DeformationGrad = defaulttype::Mat<N,N,Real1>;  // defines the dimension of the deformation tensor


    /// Function responsible for the initialization of the component
    void init() override;

    /// Add the explicit forces (right hand side)
    void addForce (const core::MechanicalParams* params, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;

    /// Add the explicit derivatives of the forces (contributing to the right hand side vector b)
    /// IF iterative solver: add the implicit derivatives of the forces (contributing to the left hand side matrix A)
    void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df , const DataVecDeriv& d_dx) override;

    // for when the bladder is being radially inflated
    void addPressure(const sofa::core::MechanicalParams* mparams, DataVecDeriv1& P, DataVecDeriv2& Patm, \
                      const DataVecCoord1& x1, const DataVecCoord2& x2, const DataVecDeriv1& v1, const DataVecDeriv2& v2) override;

    void reinit() override;

    void draw(const core::visual::VisualParams* vparams) override;



    /// IF direct solver: add the implicit derivatives of the forces (contributing to the left hand side matrix A)
    void addKToMatrix(sofa::defaulttype::BaseMatrix *m, SReal kFactor, unsigned int &offset) override;

    /// Same as previous, but using accessor
    void addKToMatrix(const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/, SReal /*kFact*/) ;

    SReal getPotentialEnergy(const core::MechanicalParams* params, const DataVecCoord& x) const override;

protected:

    IsochoricForceField();
    ~IsochoricForceField();

};


#if  !defined(SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_CPP)
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec3Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec2Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec1Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec6Types>;
#endif


} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_H
