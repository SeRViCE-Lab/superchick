#ifndef SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_H

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
*  Ripped off sofa/modules/SofaMiscFem/TetrahedronMooneyRivlinFEMForceField.h
*
* Author: Lekan Ogunmolu, December 18, 2019
*/

#include <map>
#include <string>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/MatSym.h>
#include <sofa/core/behavior/ForceField.h>
#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <IABPlugin/ForceFields/include/initIABPlugin.h>
#include <IABPlugin/ForceFields/include/NonlinearElasticMaterial.h>

// simple macro for mathematical ops
inline double SQ(double x){ return std::pow(x, 2.0); }
inline double cot(double x) {return std::cos(x)/std::sin(x); }
inline double csc(double x) {return 1.0/std::sin(x); }
inline double deg2rad(double x) {return M_PI * (x/180); }



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

//***************** Tetrahedron FEM code for several elastic models: TotalLagrangianForceField************************//

/** Compute Finite Element forces based on tetrahedral elements.
*/
template<typename DataTypes>
class TetrahedronMooneyRivlinFEMForceField : public core::behavior::ForceField<DataTypes>
{
  public:
    SOFA_CLASS(SOFA_TEMPLATE(TetrahedronMooneyRivlinFEMForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));
    using Inherited = core::behavior::ForceField<DataTypes>;
    using Real =  typename DataTypes::Real;
    typedef typename DataTypes::Coord Coord; // basically Vec3Types as defined in .cpp file
    // using Coord = typename DataTypes::Coord; // typename
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
    using SetAnisotropyDirectionArray = helper::vector<Coord>;

    using Index =  core::topology::BaseMeshTopology::index_type;
    using Element =  core::topology::BaseMeshTopology::Tetra;
    using VecElement =  core::topology::BaseMeshTopology::SeqTetrahedra;
    using Tetrahedron =  sofa::core::topology::Topology::Tetrahedron;
    using TetraID =  sofa::core::topology::Topology::TetraID;
    using Tetra =  sofa::core::topology::Topology::Tetra;

    // Edge Info
    using Edge =  sofa::core::topology::Topology::Edge;
    using EdgesInTriangle =  sofa::core::topology::BaseMeshTopology::EdgesInTriangle;
    using EdgesInTetrahedron =  sofa::core::topology::BaseMeshTopology::EdgesInTetrahedron;
    using TrianglesInTetrahedron =  sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron;

    sofa::component::fem::MaterialParameters<DataTypes> globalParameters;
    /*Why are these vectors */
    Data<helper::vector<Real> > f_poisson; ///< Poisson ratio in Hooke's law (vector)
    Data<helper::vector<Real> > f_young; ///< Young modulus in Hooke's law (vector)
    Data<Real> f_damping; ///< Ratio damping/stiffness

    void setMaterialName(const string name) {
        d_materialName.setValue(name);
    }
    void setparameter(const vector<Real> param) {
        d_parameterSet.setValue(param);
    }
    void setdirection(const vector<Coord> direction) {
        d_anisotropySet.setValue(direction);
    }

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

  	class MatrixList
  	{
    	public:
    		Matrix3 data[6];
  	};


    struct SphericalPolarInfoLagrangean
    {
      Real m_R, m_Theta, m_Phi;
      // internal and external radii
      Real m_Ri, m_Ro ;
      // position vector eulerian and lagrangean forms
      Coord m_RadialVector; // [3] [3];
      //fiber direction Lagrangean conf
      Coord m_M[3];
      Real m_lambda_R, m_lambda_Theta, m_lambda_Phi;
      // angle between two fibers::eulerian and lagrangrean forms
      Real m_Alpha, m_Beta, Gamma; // gamma is the diff. between two fibers
      // angle of shear between configurations
      Real m_Shear;
      // fiber Vector in Eulerian coordinates
      Coord m_fiberDirection;
      Real J;
    };

    struct SphericalPolarInfo
    {
      // position vector eulerian and lagrangean forms
      Coord m_radialVector; // [3] [3];
      // internal and external radii
      Real m_ri, m_ro ;
      /// fiber direction in eulerian configuration: vector M
      Coord m_m[3];
      // components in spherical polar coordinates
      Real m_r, m_theta, m_phi;
      /// deformation gradient = F
      Matrix3 m_F; //(0); // initialize to zeros
      /// right Cauchy-Green deformation tensor C (gradPhi^T gradPhi)
      Matrix3 m_C; //(0); // initialize to zeros
      // left Cauchy-Green Tensor
      Matrix3 m_B; //(0); // initialize to zeros
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
      Real J;
    };

    /// data structure stored for each tetrahedron
  	class TetrahedronRestInformation : public sofa::component::fem::StrainInformation<DataTypes>
    {
        public:
          /// shape vector at the rest configuration
          Coord m_shapeVector[4];
          // define a spherical polar point for each of the 4 vertices of the tetrahedron
          helper::vector<SphericalPolarInfoLagrangean> m_sPolarVecLag; // Lagrangian Coordinates
          helper::vector<SphericalPolarInfo> m_sPolarVecEul; // Eulerian Coordinates
          // volume
          Real m_restVolume, m_volScale, m_volume; // do we need this
          // Coord m_fiberDirection;
          // Mooney-Rivlin Constants
          Real m_C1, m_C2;
          //
          MatrixSym m_SPKTensorGeneral;
          /// deformation gradient = gradPhi
          // right and left Cauchy-Green Tensors
          Matrix3 rightCauchyGreen, leftCauchyGreen;
          Matrix3 m_deformationGradient;
          // MatrixSym m_deformationGradient;
          Real J;
          Real m_strainEnergy;

          /// Output stream
          inline friend ostream& operator<< ( ostream& os, const TetrahedronRestInformation& /*eri*/ ) {  return os;  }
          /// Input stream
          inline friend istream& operator>> ( istream& in, TetrahedronRestInformation& /*eri*/ ) { return in; }

          TetrahedronRestInformation() {}
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

    class SOFA_IABPlugin_API TetrahedronHandler: public TopologyDataHandler<Tetrahedron,sofa::helper::vector<TetrahedronRestInformation> >
    // class  TetrahedronHandler: public TopologyDataHandler<Tetrahedron,sofa::helper::vector<TetrahedronRestInformation> >
    {
      public:
        using TetrahedronRestInformation = TetrahedronMooneyRivlinFEMForceField<DataTypes>::TetrahedronRestInformation ;
        TetrahedronHandler(TetrahedronMooneyRivlinFEMForceField<DataTypes>* ff,
                           TetrahedronData<sofa::helper::vector<TetrahedronRestInformation> >* data )
          :TopologyDataHandler<Tetrahedron,sofa::helper::vector<TetrahedronRestInformation> >(data)
          ,ff(ff)
        {

        }
        void applyCreateFunction(unsigned int, TetrahedronRestInformation &t, const Tetrahedron &,
                                 const sofa::helper::vector<unsigned int> &, const sofa::helper::vector<double> &);

      protected:
        TetrahedronMooneyRivlinFEMForceField<DataTypes>* ff;
    };

    /* Member Functions */
    void init() override;
    virtual void reinit() override;
    void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v) override;
    void addDForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& d_df, const DataVecDeriv& d_dx) override;
    SReal getPotentialEnergy(const core::MechanicalParams*, const DataVecCoord&) const override;
    void addKToMatrix(sofa::defaulttype::BaseMatrix *mat, SReal k, unsigned int &offset) override;
    void draw(const core::visual::VisualParams* vparams) override;
    Mat<3,3,double> get_defGrad( int tetrahedronIndex);

  protected:
    core::topology::BaseMeshTopology* m_topology;
    VecCoord  m_initialPoints;	/// the intial positions of the points
    bool m_updateMatrix;
    bool  m_meshSaved ;

    Data<bool> d_stiffnessMatrixRegularizationWeight; ///< Regularization of the Stiffness Matrix (between true or false)
    Data<string> d_materialName; ///< the name of the material
    Data<SetParameterArray> d_parameterSet; ///< The global parameters specifying the material
    Data<SetAnisotropyDirectionArray> d_anisotropySet; ///< The global directions of anisotropy of the material

    TetrahedronData<sofa::helper::vector<TetrahedronRestInformation> > m_tetrahedronInfo; ///< Internal tetrahedron data
    EdgeData<sofa::helper::vector<EdgeInformation> > m_edgeInfo; ///< Internal edge data
    TetrahedronMooneyRivlinFEMForceField();
    virtual ~TetrahedronMooneyRivlinFEMForceField();
    /// the array that describes the complete material energy and its derivatives
    fem::NonlinearElasticMaterial<DataTypes> *m_MRIncompMatlModel;
    TetrahedronHandler* m_tetrahedronHandler;
    void testDerivatives();
    void saveMesh( const char *filename );
    void updateTangentMatrix();
};

using defaulttype::Vec3dTypes;

#if  !defined(SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_CPP)
using sofa::defaulttype::Vec3Types;
extern template class SOFA_BOUNDARY_CONDITION_API TetrahedronMooneyRivlinFEMForceField<Vec3Types>;
// extern template class SOFA_BOUNDARY_CONDITION_API TetrahedronMooneyRivlinFEMForceField<sofa::defaulttype::Vec2Types>;
// extern template class SOFA_BOUNDARY_CONDITION_API TetrahedronMooneyRivlinFEMForceField<sofa::defaulttype::Vec1Types>;
// extern template class SOFA_BOUNDARY_CONDITION_API TetrahedronMooneyRivlinFEMForceField<sofa::defaulttype::Vec6Types>;
extern template class SOFA_IABPlugin_API TetrahedronMooneyRivlinFEMForceField<Vec3Types>;
#endif //

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
