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

#include "IABPlugin/ForceFields/include/integrand.inl"  // will help with our integrations
#include "IABPlugin/ForceFields/include/IsochoricForceField.h"
#include <cassert>
#include <iostream>
#include <sofa/helper/rmath.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/RGBAColor.h>
#include <sofa/helper/system/config.h>

// see SoftRobots/model/SurfacePressureModel
namespace sofa
{

namespace component
{

namespace forcefield
{
// Constructor of the class IsochoricForceField
// initializing data with their default value (here d_inputForTheUser=20)
template<typename DataTypes>
IsochoricForceField<DataTypes>::IsochoricForceField()
    : indices(initData(&indices, "indices", "index of nodes controlled by the isochoric deformations")),
    d_Ri(initData(&d_Ri, Real(12), "Ri", "internal radius in the reference configuration")),
    d_Ro(initData(&d_Ro, Real(15), "Ro", "external radius in the reference configuration")),
    d_ri(initData(&d_ri, Real(20), "ri", "internal radius in the current configuration")),
    // d_ro(initData(&d_ro, Real(25), "ro", "external radius in the current configuration")),
    d_C1(initData(&d_C1, Real(1.e6), "C1", "material elasticity of the internal IAB wall")),
    d_C2(initData(&d_C2, Real(2.2e6), "C2", "material elasticity of the outer IAB wall")),
    d_mode(initData(&d_mode, "expand", "mode", "mode of deformation: <expansion> or <compression>"))
{
  //default Constructor
  init();
}


template<typename DataTypes>
IsochoricForceField<DataTypes>::~IsochoricForceField()
{
}


template<typename DataTypes>
void IsochoricForceField<DataTypes>::init()
{
    // ripped off angularSpringForceField
    core::behavior::ForceField<DataTypes>::init();

    if((d_ri.getValue()==0) && (d_ro.getValue()==0))
    {
      std::cerr << "Understand that these ri and ro values cannot be both zero" << std::endl;
      // std::terminate();
    }

    abstol = 1e-2F;
    reltol = 1e-5F;

    m_Ri = d_Ri.getValue();
    m_Ro = d_Ro.getValue();
    m_ri = d_ri.getValue();
    // m_ro = d_ro.getValue();
    m_C1 = d_C1.getValue();
    m_C2 = d_C2.getValue();

    // radii to take IAB into in the current configuration
    m_ro = std::cbrt(std::pow(m_Ro, 3) - std::pow(m_Ri, 3) + \
            std::pow(m_ri, 3));
    d_ro.setValue(m_ro);
    //  mState = dynamic_cast<core::behavior::MechanicalState<DataTypes> *> (this->getContext()->getMechanicalState());
    // if (!mState) {
		// msg_error("IsochoricForceField") << "MechanicalStateFilter has no binding MechanicalState" << "\n";
    // }
    // matS.resize(mState->getMatrixSize(), mState->getMatrixSize());
}


template<typename DataTypes>
void IsochoricForceField<DataTypes>::reinit()
{
  // not yet implemented
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                             DataVecDeriv& f,
                                             const DataVecCoord& x,
                                             const DataVecDeriv& v)
{
    if(!mState) {
    msg_info("IsochoricForceField") << "No Mechanical State found, no force will be computed..." << "\n";
        return;
    }
    // Compute the forces f from the current DOFs p; here i am using the derived stress from eq 25v in paper 1
    helper::WriteAccessor< DataVecDeriv > f1 = f;
    helper::ReadAccessor< DataVecCoord >  p1 = x;
    helper::ReadAccessor< DataVecDeriv >  v1 = v;

    f1.resize(p1.size());

    // // these from PREquivalentStiffnessForceField.inl
    //     if(m_componentstate != ComponentState::Valid)
    //         return ;

    SOFA_UNUSED(f);
    SOFA_UNUSED(x);
    SOFA_UNUSED(v);

    // const VecCoord& X = x.getValue();
    // VecDeriv& force = *f.beginEdit();
    // size_t nFrames = X.size();
    //
    // std::copy(X.begin(), X.end(), m_pos.begin());
    //
    // const Real& coef = d_coeff.getValue();
    // const unsigned int& start = d_startIndex.getValue();
    //
    // for(size_t n = 0 ; n < nFrames - (start + 1) ; ++n) {
    //     const Pos& x0Current = X[n + 0 + start].getCenter();
    //     const Pos& x1Current = X[n + 1 + start].getCenter();
    //
    //     const Pos& x0Rest = m_restPos[n + 0 + start].getCenter();
    //     const Pos& x1Rest = m_restPos[n + 1 + start].getCenter();
    //
    //     const Quaternion& q0Current = X[n + 0 + start].getOrientation();
    //     const Quaternion& q1Current = X[n + 1 + start].getOrientation();
    //     const Quaternion& q0Rest = m_restPos[n + 0 + start].getOrientation();
    //     const Quaternion& q1Rest = m_restPos[n + 1 + start].getOrientation();
    //
    //     // compute x1 local rigid position and rotation (can be precomputed)
    //     const Pos& x1l0Rest = q0Rest.inverseRotate(x1Rest - x0Rest);
    //     const Quaternion& q1l0Rest = q0Rest.inverse() * q1Rest;
    //
    //     // compute x1 position w.r.t. x0 frame
    //     const Pos& x1l0Current = q0Current.inverseRotate(x1Current - x0Current);
    //
    //     // compute the difference between rigid and real positions and orientations
    //     const Pos& dx = x1l0Rest - x1l0Current;
    //     //    Quaternion qDiff = q0Current.inverse() * q1th.inverse() * q1Current;
    //     Quaternion dummy;
    //     Quaternion q1l0  = q0Current.inverse() * q1Current;
    //     Quaternion qdiff = q1l0 * q1l0Rest.inverse();
    //     qdiff.normalize();
    //
    //     Vec3 dq = dummy.angularDisplacement(q1l0Rest, q1l0);
    //     Vec6 dX1(dx, dq);
    //
    //     // test: verification that we obtain the good values for linear and angular displacements in local frames
    //     Transform World_H_X0_rest(x0Rest,q0Rest);
    //     Transform World_H_X1_rest(x1Rest,q1Rest);
    //     Transform X0_H_X1_Rest = World_H_X0_rest.inversed() * World_H_X1_rest;
    //
    //     Transform World_H_X0_current(x0Current,q0Current);
    //     Transform World_H_X1_current(x1Current,q1Current);
    //     Transform X0_H_X1_Current = World_H_X0_current.inversed() * World_H_X1_current;
    //
    //     /// compute the angular displacement:
    //     SpatialVector UinX0= X0_H_X1_Current.CreateSpatialVector() - X0_H_X1_Rest.CreateSpatialVector();
    //     Vec6 Uloc;
    //     for (unsigned int i=0; i<3; i++)
    //     {
    //         Uloc[i] = UinX0.getLinearVelocity()[i];
    //         Uloc[i+3] = UinX0.getAngularVelocity()[i];
    //     }
    //     Vec6 Floc = m_CInv[n] * Uloc;
    //     Vec3 f_loc(Floc.ptr());
    //     Vec3 tau_loc(Floc.ptr() + 3);
    //
    //     SpatialVector F1atX1inX0(f_loc, tau_loc);
    //
    //     bool rest=true;
    //     SpatialVector F1atX1inX1,F0atX0inX0;
    //     if(rest){
    //         F1atX1inX1.setForce( X0_H_X1_Rest.backProjectVector( F1atX1inX0.getForce()) );
    //         F1atX1inX1.setTorque( X0_H_X1_Rest.backProjectVector( F1atX1inX0.getTorque()) )  ;
    //         F0atX0inX0 = X0_H_X1_Rest*F1atX1inX1;
    //     }
    //     else
    //     {
    //         F1atX1inX1.setForce( X0_H_X1_Current.backProjectVector( F1atX1inX0.getForce()) );
    //         F1atX1inX1.setTorque( X0_H_X1_Current.backProjectVector( F1atX1inX0.getTorque()) )  ;
    //         F0atX0inX0 = X0_H_X1_Current*F1atX1inX1;
    //     }
    //
    //     SpatialVector F0atX0inWorld(World_H_X0_current.projectVector(F0atX0inX0.getForce()), World_H_X0_current.projectVector(F0atX0inX0.getTorque()))  ;
    //     SpatialVector F1atX1inWorld(World_H_X0_current.projectVector(F1atX1inX0.getForce()),  World_H_X0_current.projectVector(F1atX1inX0.getTorque()));
    //
    //
    //     // compute x1 forces in x0's frame and rotate them back to global coordinates
    //     Vec6 f1l0 = m_CInv[n] * dX1;
    //
    //     Vec3 F1(f1l0.ptr());
    //     Vec3 tau1(f1l0.ptr() + 3);
    //     F1 = q0Current.rotate(F1);
    //     tau1 = q0Current.rotate(tau1);
    //
    //
    //     // compute transport matrix
    //     Vec3 p0p1;
    //     if(rest)
    //         p0p1= q0Rest.inverseRotate(x1Rest - x0Rest); // p0^p1 in local frame
    //     else
    //         p0p1= q0Current.inverseRotate(x1Current - x0Current); // p0^p1 in local frame
    //
    //     Mat66 H = Mat66::Identity();
    //     H(3, 1) = -p0p1.z();
    //     H(3, 2) = p0p1.y();
    //     H(4, 0) = p0p1.z();
    //     H(4, 2) = -p0p1.x();
    //     H(5, 0) = -p0p1.y();
    //     H(5, 1) = p0p1.x();
    //
    //
    //     H = -H;
    //
    //     // compute f0
    //     Vec6 f0l0 = H * f1l0;
    //     Vec3 F0(f0l0.ptr());
    //     Vec3 tau0(f0l0.ptr() + 3);
    //
    //     F0 = q0Current.rotate(F0);
    //     tau0 = q0Current.rotate(tau0);
    //
    //
    //     Vec6 f0( F0atX0inWorld.getForce(),  F0atX0inWorld.getTorque());
    //     Vec6 f1(-F1atX1inWorld.getForce(), -F1atX1inWorld.getTorque());
    //
    //     force[n + 0 + start] += f0 * coef;
    //     force[n + 1 + start] += f1 * coef;
    //
    //
    //     Mat66 block = H * m_CInv[n];
    //     m_K[n + start].setsub(0, 6, block);
    //     m_K[n + start].setsub(6, 0, block.transposed());
    //
    //     block =  H * m_CInv[n] * H.transposed();
    //     m_K[n + start].setsub(0, 0, block);
    //     m_K[n + start].setsub(6, 6, m_CInv[n]);
    //
    //
    //     // build rotation matrix 4 3x3 blocks on diagonal
    //     Mat33 Rn;
    //     q0Current.toMatrix(Rn);
    //     Mat12x12 R(.0);
    //     R.setsub(0, 0, Rn);
    //     R.setsub(3, 3, Rn);
    //     R.setsub(6, 6, Rn);
    //     R.setsub(9, 9, Rn);
    //
    //     m_K[n + start] = -R * m_K[n + start] * R.transposed() * coef; // modified : wasn't negated
    // }
    // f.endEdit();

    // calculate the stress and pressure needed to go from a reference configuartion to a current configuration
    auto stressFunc = radial_stress_r2c<float>(m_Ri, m_Ro, m_ri, m_C1, m_C2);
    const float stress_rr = integrator<float, \
                                radial_stress_r2c<float>>(m_ri, m_ro, \
                                      abstol, reltol, stressFunc);
    std::cout << "Calculated normal stress based on given parameters:\n\t" <<
      "[C1, C2, Ri, Ro, ri, ro]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] is \n\t"<<
      "Sigma=%.4f " << m_C1  << m_C2 << m_Ri << m_Ro <<  m_ri << m_ro << stress_rr;
}


// for when the bladder is being radially inflated
template<typename DataTypes>
void IsochoricForceField<DataTypes>::addPressure(const sofa::core::MechanicalParams* mparams, DataVecDeriv& P, DataVecDeriv& Patm, \
                  const DataVecCoord& x1, const DataVecDeriv& v1)
{
  auto PressureFunc = pressure_r2c<float>(m_Ri, m_Ro, m_ri, m_C1, m_C2);
  // integrate from ri to ro in r
  const float int_pressure = integrator<float, \
                              pressure_r2c<float>>(m_ri, m_ro, \
                                    abstol, reltol, PressureFunc);
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams,
                                              DataVecDeriv& d_df , const DataVecDeriv& d_dx)
{
    // Compute the force derivative d_df from the current, which will be multiplied with the field d_dx
}


template<typename DataTypes>
void IsochoricForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix * /* mat */,
                                                 SReal /* k */, unsigned int & /* offset */)
{
    // Compute the force derivative d_df from the current and store the resulting matrix
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
  // nothing to do here
};

template<typename DataTypes>
void IsochoricForceField<DataTypes>::addKToMatrix(const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/,
                                                 SReal /*kFact*/)
{
    // Same as previously
    // but using accessor
}


template <typename DataTypes>
SReal IsochoricForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams* mparams,
                                                        const DataVecCoord& x) const
{
  SOFA_UNUSED(mparams);
  SOFA_UNUSED(x);

  return 0.0; // dummy retun for now
}




} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_INL
