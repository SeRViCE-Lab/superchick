// Ripped off /home/lex/sofa/modules/SofaMiscFem/MooneyRivlin.h
#include <SofaMiscFem/initMiscFEM.h>
#include <IABPlugin/ForceFields/include/NonlinearElasticMaterial.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <string>


namespace sofa
{

namespace component
{

namespace fed  // finite elastic deformation namespace
{

/** a Class that describe a generic hyperelastic material : exemple of Boyce and Arruda
The material is described based on continuum mechanics and the description is independent
to any discretization method like the finite element method.
A material is generically described by a strain energy function and its first and second derivatives.
In practice the energy is the sum of several energy terms which depends on 2 quantities :
the determinant of the deformation gradient J and the right Cauchy Green deformation tensor */



template<class DataTypes>
class MooneyRivlin : public NonlinearElasticMaterial<DataTypes>{

  typedef typename DataTypes::Coord::value_type Real;
  typedef defaulttype::Mat<3,3,Real> Matrix3;
  typedef defaulttype::Mat<6,6,Real> Matrix6;
  typedef defaulttype::MatSym<3,Real> MatrixSym;

  virtual Real getStrainEnergy(StrainInformation<DataTypes> *sinfo, const MaterialParameters<DataTypes> &param) {
	  MatrixSym Finv;
		MatrixSym F=sinfo->deformationTensor;
		invertMatrix(Finv,F);
		Real I1=sinfo->trF; // or (Real) trace(F);
		// Real I1square=(Real)(F[0]*F[0] + F[2]*F[2]+ F[5]*F[5]+2*(F[1]*F[1] + F[3]*F[3] + F[4]*F[4]));
		// Real I2=(Real)((pow(I1,(Real)2)- I1square)/2);
    Real I2=(Real)trace(Finv);
		Real C1=param.parameterArray[0];
		Real C2=param.parameterArray[1];
		// Real k0=param.parameterArray[2];
		// return C1*(I1*pow(sinfo->J,(-2/3))-3)+C2*(I2*pow(sinfo->J,(-4/3)))+k0*log(sinfo->J)*log(sinfo->J)/2;
    return .5*C1*(I1-3) + .5*C2*(I2-3);
  }

  virtual MatrixSym getStressTensor(StrainInformation<DataTypes> *sinfo, const  MaterialParameters<DataTypes> &param) {

    Real C1=param.parameterArray[0];
    Real C2=param.parameterArray[1];
    Matrix3 identityMatrix = Identity();
    MatrixSym rightCauchyInv;
    invertMatrix(rightCauchyInv, sinfo->rightCauchy);
    return C1*sinfo->leftCauchy-C2*rightCauchyInv-hydrostaticPressure*identityMatrix);
  }

  virtual MatSym PiolaKirchoffTensor(StrainInformation<DataTypes> *, const  MaterialParameters<DataTypes> &,MatrixSym &){
      // S = J H^T \sigma
      MatrixSym F=sinfo->deformationTensor;
      MatrixSym Finv;
      invertMatrix(Finv,F);
      H = Finv.transposed();
      Real J = determinant(F);
      MatrixSym PiolaKirchoff = J * H.transposed() * this->getStressTensor(sinfo, params);
      return PiolaKirchoff;
  }
};


} // namespace fem

} // namespace component

} // namespace sofa

#endif
