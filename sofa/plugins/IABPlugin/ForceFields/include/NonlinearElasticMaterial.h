/*
* Tetrahedron components ripped off but adapted to incompressible materials
*  Ripped off sofa/modules/SofaMiscFem/Hyperelastic.h
*
* Author: Lekan Ogunmolux, December 18, 2019
*/
#ifndef SOFA_COMPONENT_FEM_NONLINEARELASTICMATERIAL_H
#define SOFA_COMPONENT_FEM_NONLINEARELASTICMATERIAL_H
#include "config.h"


#include <sofa/core/topology/BaseMeshTopology.h>
#include <SofaBaseTopology/TopologyData.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/MatSym.h>
#include <string>

//#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
namespace sofa
{

namespace component
{

namespace fem
{

template<typename Real>
class StrainInformation;

template<typename DataTypes>
struct MaterialParameters;

/** a Class that describe a generic hyperelastic material .
The material is described based on continuum mechanics and the description is independent
to any discretization method like the finite element method.
A material is generically described by a strain energy function and its first and second derivatives.
*/
template<class DataTypes>
class NonlinearElasticMaterial
{
public:

  using Coord = typename DataTypes::Coord;
  using Real = typename Coord::value_type;
  using MatrixSym = defaulttype::MatSym<3,Real>;
  using Matrix3 = defaulttype::Mat<3,3,Real>;
  using Matrix6 = defaulttype::Mat<6,6,Real>;
  using Line = defaulttype::Vec<3,Real>;
  // using Vec<3,real> dummyLines{0,0,0};

   virtual ~NonlinearElasticMaterial(){}
   Real hydrostaticPressure=0; // should cancel out in equation anyway


	/** returns the strain energy of the current configuration */
	virtual Real getStrainEnergy(StrainInformation<DataTypes> *, const  MaterialParameters<DataTypes> &) {
			return 0;
	}

  /** returns the strain energy of the current configuration */
	virtual Matrix3 getCauchyStressTensor(StrainInformation<DataTypes> *, const  MaterialParameters<DataTypes> &) {
      // initialize from dummy lines
      Line dummyLines{0,0,0};
      return Matrix3(dummyLines, dummyLines, dummyLines);
	}
  // /** returns the stress tensor of Cauchy of the current configuration */
	// virtual Real getCauchyStressTensor(StressInformation<DataTypes> *, const  MaterialParameters<DataTypes> &) {
	// 		return 0;
	// }

	/** computes the second Piola Kirchhoff stress tensor of the current configuration */
    virtual void deriveSPKTensor(StrainInformation<DataTypes> *, const  MaterialParameters<DataTypes> &,MatrixSym &)  {

	}
	/** computes the Elasticity Tensor of the current configuration
      Why are we using the Mooney-Rivlin form for a solid here?
  */

    virtual void applyElasticityTensor(StrainInformation<DataTypes> *sinfo, const  MaterialParameters<DataTypes> &param,const MatrixSym& inputTensor, MatrixSym &outputTensor)  {
  		MatrixSym inversematrix;
  		MatrixSym F=sinfo->deformationTensor;
  		invertMatrix(inversematrix,F);
  		Real I1=trace(F);
  		Real I1square=(Real)(F[0]*F[0] + F[2]*F[2]+ F[5]*F[5]+2*(F[1]*F[1] + F[3]*F[3] + F[4]*F[4]));
  		Real I2=(Real)((pow(I1,(Real)2)- I1square)/2);
  		Real c1=param.parameterArray[0];
  		Real c2=param.parameterArray[1];
  		Real k0=param.parameterArray[2];
  		MatrixSym ID;
  		ID.identity();
  		// C-1:H
  		Real _trHC=inputTensor[0]*inversematrix[0]+inputTensor[2]*inversematrix[2]+inputTensor[5]*inversematrix[5]
  		+2*inputTensor[1]*inversematrix[1]+2*inputTensor[3]*inversematrix[3]+2*inputTensor[4]*inversematrix[4];
  		MatrixSym Firstmatrix;
  		//C-1HC-1 convert to sym matrix
  		Firstmatrix.Mat2Sym(inversematrix.SymMatMultiply(inputTensor.SymSymMultiply(inversematrix)),Firstmatrix);
  		//C:H
  		Real trHC=inputTensor[0]*F[0]+inputTensor[2]*F[2]+inputTensor[5]*F[5]
  		+2*inputTensor[1]*F[1]+2*inputTensor[3]*F[3]+2*inputTensor[4]*F[4];

  		//trH
  		Real trH=inputTensor[0]+inputTensor[2]+inputTensor[5];

  		outputTensor=((ID-inversematrix*I1/(Real)3.0)*(-_trHC)/(Real)3.0+Firstmatrix*I1/(Real)3.0-inversematrix*trH/(Real)3.0)*(Real)2.0*c1*pow(sinfo->J,(Real)(-2.0/3.0))
  			+((inversematrix*(Real)(-2.0)*I2/(Real)3.0+ID*I1-F)*(Real)(-2.0)*_trHC/(Real)3.0+Firstmatrix*(Real)2.0*I2/(Real)3.0-inversematrix*(Real)2.0*(I1*trH-trHC)/(Real)3.0+ID*trH-inputTensor)*(Real)2.0*c2*pow(sinfo->J,(Real)(-4.0/3.0))
  			+inversematrix*_trHC*k0/(Real)2.0-Firstmatrix*k0*log(sinfo->J);

	}

	virtual void ElasticityTensor(StrainInformation<DataTypes> *, const  MaterialParameters<DataTypes> &, Matrix6&) {;}

};

/** structure that store the parameters required to that are necessary to compute the strain energy
The material parameters might be constant in space (homogeneous material) or not */
template<typename DataTypes>
struct MaterialParameters {
  typedef typename DataTypes::Coord Coord;
  typedef typename Coord::value_type Real;

  /** an array of Real values that correspond to the material parameters : the size depends on the material,
  e.g. 2 Lame coefficients for St-Venant Kirchhoff materials */
  std::vector<Real> parameterArray;
  /** the direction of anisotropy in the rest configuration  : the size of the array is 0 if the material is
  isotropic, 1 if it is transversely isotropic and 2 for orthotropic materials (assumed to be orthogonal to each other)*/
  std::vector<Coord> anisotropyDirection;
  /** for viscous part, give the real alphai and taui such as alpha(t)= alpha0+sum(1,N)alphaiexp(-t/taui)*/
  std::vector<Real> parameterAlpha;
  std::vector<Real> parameterTau;//starting with delta t the time step
};

template<typename DataTypes>
class StrainInformation
{
public:


  typedef typename DataTypes::Coord Coord;
  typedef typename Coord::value_type Real;
  typedef defaulttype::MatSym<3,Real> MatrixSym;
  typedef typename Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Real,3,3> >::MatrixType EigenMatrix;
  typedef typename Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Real,3,3> >::RealVectorType CoordEigen;
  /// Trace of C = I1
  Real trC;
  // Real trF;
  Real J; // det (A)
  Real lambda; // stretch ratios
  /// Trace of C^2 : I2 = (trCSquare - trC^2)/2
  Real trCsquare;

  /// boolean indicating whether the invariants have been computed
  bool hasBeenInitialized;
  /// right Cauchy-Green deformation tensor C (gradPhi^T gradPhi)
  MatrixSym deformationTensor;
  // MatrixSym leftCauchy, rightCauchy;
  MatrixSym F; // deformation gradient
  EigenMatrix Evect;
  CoordEigen Evalue;
  Real logJ;
  MatrixSym E;


  StrainInformation() : trC(0), J(0), lambda(0), trCsquare(0), hasBeenInitialized(false), deformationTensor(), Evect(), Evalue(), logJ(0), E() {}
  virtual ~StrainInformation() {}
};

} // namespace fem

} // namespace component

} // namespace sofa

#endif
