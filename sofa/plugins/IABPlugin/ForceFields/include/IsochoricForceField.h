#ifndef SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_H

#include "IABPlugin/include/debuggers.h"  // this from global project include
#include "IABPlugin/ForceFields/include/config.h"

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/MechanicalParams.h>
#include <SofaEigen2Solver/EigenSparseMatrix.h>
/*
* Tetrahedron components ripped off but adapted to incompressible materials
*  Ripped off sofa/modules/SofaMiscFem/TetrahedronHyperelasticityFEMForceField.h
*
* Author: Lekan Ogunmolux, December 18, 2019
*/

#include <sofa/defaulttype/RGBAColor.h>
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
    using SetAnisotropyDirectionArray =  helper::vector<Coord>;

    using Index =  core::topology::BaseMeshTopology::index_type;
    using Element =  core::topology::BaseMeshTopology::Tetra;
    using VecElement =  core::topology::BaseMeshTopology::SeqTetrahedra;
    using Tetrahedron =  sofa::core::topology::Topology::Tetrahedron;
    using TetraID =  sofa::core::topology::Topology::TetraID;
    using Tetra =  sofa::core::topology::Topology::Tetra;
    using Edge =  sofa::core::topology::Topology::Edge;
    using EdgesInTriangle =  sofa::core::topology::BaseMeshTopology::EdgesInTriangle;
    using EdgesInTetrahedron =  sofa::core::topology::BaseMeshTopology::EdgesInTetrahedron;
    using TrianglesInTetrahedron =  sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron;


  public :
    	typename sofa::component::fem::MaterialParameters<DataTypes> globalParameters;

    /// data structure stored for each tetrahedron
	class TetrahedronRestInformation : public sofa::component::fem::StrainInformation<DataTypes>
  {
    public:
        /// shape vector at the rest configuration
        Coord m_shapeVector[4];
        /// fiber direction in rest configuration
        Coord m_fiberDirection;
        /// rest volume
        Real m_restVolume;
        /// current tetrahedron volume
        Real m_volScale;
        Real m_volume;
        /// volume/ restVolume
        MatrixSym m_SPKTensorGeneral;
        /// deformation gradient = F
        Matrix3 m_deformationGradient;
        /// right Cauchy-Green deformation tensor C (gradPhi^T gradPhi)
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

   protected :
      sofa::core::topology::BaseMeshTopology* m_topology;
      VecCoord  m_initialPoints;	/// the intial positions of the points
      bool m_updateMatrix;
      bool  m_meshSaved ;

      Data<bool> d_stiffnessMatrixRegularizationWeight; ///< Regularization of the Stiffness Matrix (between true or false)
      Data<string> d_materialName; ///< the name of the material // this should be default in my case
      Data<SetParameterArray> d_parameterSet; ///< The global parameters specifying the material: C1, C2 and bulk modulus
      Data<SetAnisotropyDirectionArray> d_anisotropySet; ///< The global directions of anisotropy of the material

      TetrahedronData<sofa::helper::vector<TetrahedronRestInformation> > m_tetrahedronInfo; ///< Internal tetrahedron data from TopologyData.h
      EdgeData<sofa::helper::vector<EdgeInformation> > m_edgeInfo; ///< Internal edge data

public:

    void setparameter(const vector<Real> param) {
        d_parameterSet.setValue(param);
    }

    void setdirection(const vector<Coord> direction) {
        d_anisotropySet.setValue(direction);
    }

    class SOFA_MISC_FEM_API TetrahedronHandler : public TopologyDataHandler<Tetrahedron,sofa::helper::vector<TetrahedronRestInformation> >
    {
    public:
      using TetrahedronRestInformation = typename IsochoricForceField<DataTypes>::TetrahedronRestInformation;
      TetrahedronHandler(IsochoricForceField<DataTypes>* ff,
                         TetrahedronData<sofa::helper::vector<TetrahedronRestInformation> >* data )
        :TopologyDataHandler<Tetrahedron,sofa::helper::vector<TetrahedronRestInformation> >(data)
        ,ff(ff)
      {

      }

      void applyCreateFunction(unsigned int, TetrahedronRestInformation &t, const Tetrahedron &,
                               const sofa::helper::vector<unsigned int> &,
                               const sofa::helper::vector<double> &);

    protected:
      IsochoricForceField<DataTypes>* ff;
    };

protected:
   IsochoricForceField();
   virtual   ~IsochoricForceField();
/*
    /// Declare here the data and their type, you want the user to have access to
    Data< Real > d_Ri, d_Ro; // referencce configuration radius
    Data< Real > d_ri, d_ro; // current configuration radius
    Data< std::string > d_mode; // mode tells whether we are expanding or compressing the IABs; accepts "compress" or "expand"
    // Data< defaulttype::RGBAColor> color; ///< isochoric spherical forcefield color. (default=[0.0,0.5,1.0,1.0])
    Data< helper::vector< unsigned int > > indices; ///< index of nodes controlled by the isochoric fields
    unsigned counter = 0;

    Real m_Ri, m_Ro, m_ri, m_ro, m_C1, m_C2;  // local for internal data usage

    enum { N=DataTypes::spatial_dimensions };
    using DeformationGrad = defaulttype::Mat<N,N,Real>;  // defines the dimension of the deformation tensor
*/
public:
    /// Function responsible for the initialization of the component
    void init() override;

    virtual void reinit() override;

    /// Add the explicit forces (right hand side)
    virtual void addForce(const core::MechanicalParams* params, DataVecDeriv& d_f, const DataVecCoord& x, const DataVecDeriv& d_v) override;

    /// Add the explicit derivatives of the forces (contributing to the right hand side vector b)
    /// IF iterative solver: add the implicit derivatives of the forces (contributing to the left hand side matrix A)
    virtual void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df , const DataVecDeriv& d_dx) override;
    SReal getPotentialEnergy(const core::MechanicalParams* params, const DataVecCoord& x) const override;
    /// IF direct solver: add the implicit derivatives of the forces (contributing to the left hand side matrix A)
    void addKToMatrix(sofa::defaulttype::BaseMatrix *m, SReal kFactor, unsigned int &offset) override;
    /// Same as previous, but using accessor
    void addKToMatrix(const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/, SReal /*kFact*/) ;
    void draw(const core::visual::VisualParams* vparams) override;
    Mat<3,3,double> getF( int tetrahedronIndex); // deformation Grad at the index tetrahedronIndex
    // linearsolver::EigenBaseSparseMatrix<typename DataTypes::Real> matS;

protected:
    /// the array that describes the complete material energy and its derivatives
    sofa::component::fem::NonlinearMaterial<DataTypes> *m_MRIncompMatlModel;
    TetrahedronHandler* m_TetrahedronHandler;

    void testDerivatives();
    void saveMesh( const char *filename );
    void updateTangentMatrix();

  core::behavior::MechanicalState<DataTypes> *mState;
  // Data< Real > d_C1, d_C2; // material elasticity weights in strain energy equations
  // Real abstol, reltol;
};

#if  !defined(SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_CPP)
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec3Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec2Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec1Types>;
extern template class SOFA_BOUNDARY_CONDITION_API IsochoricForceField<sofa::defaulttype::Vec6Types>;
/*Don't think I should include this here*/
extern template class SOFA_MISC_FEM_API TetrahedronNonlinearElasticityFEMForceField<Vec3Types>;
#endif
} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_TEMPLATEFORCEFIELD_H
