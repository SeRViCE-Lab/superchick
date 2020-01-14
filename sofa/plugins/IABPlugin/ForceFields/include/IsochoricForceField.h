#ifndef SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_H

#include "IABPlugin/include/debuggers.h"  // this from global project include
#include "IABPlugin/ForceFields/include/config.h"

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/MechanicalParams.h>
#include <SofaEigen2Solver/EigenSparseMatrix.h>
#include <sofa/defaulttype/RGBAColor.h>
/*
* Tetrahedron components ripped off but adapted to incompressible materials
*  Ripped off sofa/modules/SofaMiscFem/TetrahedronHyperelasticityFEMForceField.h
*
* Author: Lekan Ogunmolux, December 18, 2019
*/

#include <IABPlugin/ForceFields/include/NonlinearElasticMaterial.h>
#include <sofa/core/behavior/ForceField.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/MatSym.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <SofaBaseTopology/TopologyData.h>
#include <string>
#include <map>

// simple macro for mathematical ops
//#define power(SQ, X) double SQ(X) {return std::pow(X, 2.0f); }
inline float SQ(float x){ return std::pow(x, 2.0f); }
inline double SQ(double x){ return std::pow(x, 2.0d); }


namespace sofa
{

namespace component
{

namespace forcefield
{
using namespace std;
using namespace sofa::defaulttype;
using namespace sofa::component::topology;
using namespace sofa::core::topology;

/// Apply constant forces to given degrees of freedom.
// DataTypes will be Vec3Types. See sofa/v19.06/SofaKernel/framework/sofa/defaulttype/VecTypes.h
template<typename DataTypes>
class IsochoricForceField : public core::behavior::ForceField<DataTypes>
{
  public:
    SOFA_CLASS(SOFA_TEMPLATE(IsochoricForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));
    using Inherited = core::behavior::ForceField<DataTypes>;
    using Real =  typename DataTypes::Real;
    using Coord = typename DataTypes::Coord;
    using Deriv = typename DataTypes::Deriv;

    using VecCoord = typename DataTypes::VecCoord;
    using VecDeriv = typename DataTypes::VecDeriv;
    using DataVecCoord = Data<VecCoord>;
    using DataVecDeriv = Data<VecDeriv>;

    using Matrix3 =  Mat<3,3,Real>;
    using MatrixSym =  MatSym<3,Real>;
    using MatrixPair =  std::pair<MatrixSym,MatrixSym>;
    using MatrixCoeffPair =  std::pair<Real,MatrixSym>;

    using SetParameterArray =  helper::vector<Real>;

    using Index =  core::topology::BaseMeshTopology::index_type;
    using Element =  core::topology::BaseMeshTopology::Tetra;
    using VecElement =  core::topology::BaseMeshTopology::SeqTetrahedra;
    using Tetrahedron =  sofa::core::topology::Topology::Tetrahedron;
    using TetraID =  sofa::core::topology::Topology::TetraID;
    using Tetra =  sofa::core::topology::Topology::Tetra;
    // declare triangle topologies for spherical points
    // since the spheres are xtized by three coordinates
    using Sphere = sofa::core::topology::Topology::Triangle;
    using SphereID = sofa::core::topology::Topology::TriangleID;
    // Fwd Declaration
    class SphericalPolarRestInformation;
    // trick for spherical polardata
    using SphericalPolarData = TriangleData<sofa::helper::vector<SphericalPolarRestInformation>>;

    // Edge Info
    using Edge =  sofa::core::topology::Topology::Edge;
    using EdgesInTriangle =  sofa::core::topology::BaseMeshTopology::EdgesInTriangle;
    using EdgesInTetrahedron =  sofa::core::topology::BaseMeshTopology::EdgesInTetrahedron;
    using TrianglesInTetrahedron =  sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron;

  public :
    	typename sofa::component::fem::MaterialParameters<DataTypes> globalParameters;
      /*Why are these vectors */
      Data<helper::vector<Real> > f_poisson; ///< Poisson ratio in Hooke's law (vector)
      Data<helper::vector<Real> > f_young; ///< Young modulus in Hooke's law (vector)
      Data<Real> f_damping; ///< Ratio damping/stiffness

  /// data structure stored for each minisphere==>Triangle
	class SphericalPolarRestInformation : public sofa::component::fem::StrainInformation<DataTypes>
  {
    public:
        // position vector eulerian and lagrangean forms
        Coord m_radialVector, m_RadialVector; // [3] [3];
        /// fiber direction in eulerian configuration: vector M
        Coord m_m[3];
        //fiber direction Lagrangean conf
        Coord m_M[3];
        // components in spherical polar coordinates
        Real m_r, m_theta, m_phi;
        // internal and external radii
        Real m_ri, m_ro ;
        // Lagrangean coordinates
        Real m_R, m_Theta, m_Phi; // see globalParameters
        Real m_Ri, m_Ro; // see globalParameters
        /// deformation gradient = F
        Matrix3 m_F; //(0); // initialize to zeros
        /// right Cauchy-Green deformation tensor C (gradPhi^T gradPhi)
        Matrix3 m_C; //(0); // initialize to zeros
        // left Cauchy-Green Tensor
        Matrix3 m_B; //(0); // initialize to zeros
        // Mooney-Rivlin Constants
        Real m_C1, m_C2;
        // Extension ratios
        Real m_lambda_r, m_lambda_theta, m_lambda_phi;
        // Cauchy Stress Tensor
        Matrix3 m_cauchyStressTensor; //(0); // initialize to zeros
        // angle between two fibers::eulerian and lagrangrean forms
        Real m_alpha, m_beta, gamma; // gamma is the diff. between two fibers
        // angle of shear between configurations
        Real m_shear;
        // fiber Vector in Eulerian coordinates
        Coord m_fiberDirection;
        // volume
        Real m_restVolume; // do we need this
        /// Output stream
        inline friend ostream& operator<< ( ostream& os, const SphericalPolarRestInformation& /*eri*/ ) {  return os;  }
        /// Input stream
        inline friend istream& operator>> ( istream& in, SphericalPolarRestInformation& /*eri*/ ) { return in; }

        SphericalPolarRestInformation() {}
  };
  /// data structure stored for each edge
  class EdgeInformation
  {
  public:
      /// store the stiffness edge matrix
      Matrix3 DfDx;
      /// Output stream
      inline friend ostream& operator<< ( ostream& os, const EdgeInformation& /*eri*/ ) {  return os;  }
      /// Input stream
      inline friend istream& operator>> ( istream& in, EdgeInformation& /*eri*/ ) { return in; }

      EdgeInformation() {}
  };

  // protected members for Isochoric Force Field
  protected :
      sofa::core::topology::BaseMeshTopology* m_topology;
      VecCoord  m_initialPoints;	/// the intial positions of the points
      bool m_updateMatrix;
      bool  m_meshSaved ;

      Data<bool> d_stiffnessMatrixRegularizationWeight; ///< Regularization of the Stiffness Matrix (between true or false)
      Data<string> d_materialName; ///< the name of the material // this should be default in my case
      /* The global parameter array should contain the ff:
          initial Ri, Ro, R, Phi, Theta, C1, C2
      */
      Data<SetParameterArray> d_parameterSet; ///< The global parameters specifying the material: C1, C2 and bulk modulus

      SphericalPolarData m_sphericalPolarInfo;
      // TriangleData<sofa::helper::vector<SphericalPolarRestInformation> > m_sphericalPolarInfo; ///< Internal tetrahedron data from TopologyData.h
      EdgeData<sofa::helper::vector<EdgeInformation> > m_edgeInfo; ///< Internal edge data
  public:
    void setparameter(const vector<Real> param) { d_parameterSet.setValue(param); }

    /// Get/Set methods
    Real getPoisson() { return (f_poisson.getValue())[0]; }
    void setPoisson(Real val)
    {
      helper::vector<Real> newP(1, val);
      f_poisson.setValue(newP);
    }
    Real getYoung() { return (f_young.getValue())[0]; }
    void setYoung(Real val)
    {
      helper::vector<Real> newY(1, val);
      f_young.setValue(newY);
    }
    Real getDamping() { return f_damping.getValue(); }
    void setDamping(Real val) { f_damping.setValue(val); }

  class SOFA_IABPlugin_API SphericalPolarHandler : public TopologyDataHandler<Sphere,sofa::helper::vector<SphericalPolarRestInformation> >
  {
  public:
    using SphericalPolarRestInformation = typename IsochoricForceField<DataTypes>::SphericalPolarRestInformation;
    SphericalPolarHandler(IsochoricForceField<DataTypes>* ff, TriangleData<sofa::helper::vector<SphericalPolarRestInformation> >* data )
      :TopologyDataHandler<Sphere,sofa::helper::vector<SphericalPolarRestInformation> >(data), ff(ff)
    {  }
    void applyCreateFunction(unsigned int, SphericalPolarRestInformation &t,
                             const Sphere &,
                             const sofa::helper::vector<unsigned int> &,
                             const sofa::helper::vector<double> &);
    protected:
        IsochoricForceField<DataTypes>* ff;
  };
  protected:
   IsochoricForceField();
   virtual   ~IsochoricForceField();

public:
    /// Function responsible for the initialization of the component
    void init() override;
    virtual void reinit() override;
    /// Add the explicit forces (right hand side)
    void addForce(const core::MechanicalParams* params, DataVecDeriv& d_f, const DataVecCoord& x, const DataVecDeriv& d_v) override;
    /// Add the explicit derivatives of the forces (contributing to the right hand side vector b)
    /// IF iterative solver: add the implicit derivatives of the forces (contributing to the left hand side matrix A)
    void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df , const DataVecDeriv& d_dx) override;
    SReal getPotentialEnergy(const core::MechanicalParams* params, const DataVecCoord& x) const override;
    /// IF direct solver: add the implicit derivatives of the forces (contributing to the left hand side matrix A)
    void addKToMatrix(sofa::defaulttype::BaseMatrix *m, SReal kFactor, unsigned int &offset) override;
    /// Same as previous, but using accessor
    void addKToMatrix(const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/, SReal /*kFact*/) ;
    void draw(const core::visual::VisualParams* vparams) override;
    void updateStrainInfo(const SphericalPolarRestInformation& sphInfo); // deformation Grad at the index tetrahedronIndex
    void computePositionalVector();
    // linearsolver::EigenBaseSparseMatrix<typename DataTypes::Real> matS;
protected:
    /// the array that describes the complete material energy and its derivatives
    fem::NonlinearElasticMaterial<DataTypes> *m_MRIncompMatlModel;
    SphericalPolarHandler* m_sphericalPolarHandler;
    // void testDerivatives();
    void saveMesh( const char *filename );
    void updateTangentMatrix();
};

#if  !defined(SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_CPP)
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec3Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec2Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec1Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec6Types>;
/*Don't think I should include this here*/
extern template class SOFA_IABPlugin_API IsochoricForceField<Vec3Types>;
#endif
} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_H
