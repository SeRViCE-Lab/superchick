/*
*  Ripped off sofa/modules/SofaMiscFem/TetrahedronHyperelasticityFEMForceField.h
*
Author: Lekan Ogunmolux, December 18, 2019
*/

#ifndef SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONNONLINEARELASTICITYFEMFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONNONLINEARELASTICITYFEMFORCEFIELD_H


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

//***************** Tetrahedron FEM code for several elastic models: TotalLagrangianForceField************************//

/** Compute Finite Element forces based on tetrahedral elements.
*/
template<class DataTypes>
class TetrahedronNonlinearElasticityFEMForceField : public core::behavior::ForceField<DataTypes>
{
  public:
    SOFA_CLASS(SOFA_TEMPLATE(TetrahedronNonlinearElasticityFEMForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherited;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename Coord::value_type Real;

    typedef core::objectmodel::Data<VecDeriv>    DataVecDeriv;
    typedef core::objectmodel::Data<VecCoord>    DataVecCoord;

    typedef Mat<3,3,Real> Matrix3;
    typedef MatSym<3,Real> MatrixSym;
    typedef std::pair<MatrixSym,MatrixSym> MatrixPair;
    typedef std::pair<Real,MatrixSym> MatrixCoeffPair;


    typedef helper::vector<Real> SetParameterArray;
    typedef helper::vector<Coord> SetAnisotropyDirectionArray;


    typedef core::topology::BaseMeshTopology::index_type Index;
    typedef core::topology::BaseMeshTopology::Tetra Element;
    typedef core::topology::BaseMeshTopology::SeqTetrahedra VecElement;
    typedef sofa::core::topology::Topology::Tetrahedron Tetrahedron;
    typedef sofa::core::topology::Topology::TetraID TetraID;
    typedef sofa::core::topology::Topology::Tetra Tetra;
    typedef sofa::core::topology::Topology::Edge Edge;
    typedef sofa::core::topology::BaseMeshTopology::EdgesInTriangle EdgesInTriangle;
    typedef sofa::core::topology::BaseMeshTopology::EdgesInTetrahedron EdgesInTetrahedron;
    typedef sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron TrianglesInTetrahedron;


public :

	typename sofa::component::fem::MaterialParameters<DataTypes> globalParameters;


	class MatrixList
	{
	public:
		Matrix3 data[6];
	};


    /// data structure stored for each tetrahedron
	class TetrahedronRestInformation : public fem::StrainInformation<DataTypes>
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
    core::topology::BaseMeshTopology* m_topology;
    VecCoord  m_initialPoints;	/// the intial positions of the points
    bool m_updateMatrix;
    bool  m_meshSaved ;

    Data<bool> d_stiffnessMatrixRegularizationWeight; ///< Regularization of the Stiffness Matrix (between true or false)
    Data<string> d_materialName; ///< the name of the material // this should be default in my case
    Data<SetParameterArray> d_parameterSet; ///< The global parameters specifying the material: C1, C2 and bulk modulus
    Data<SetAnisotropyDirectionArray> d_anisotropySet; ///< The global directions of anisotropy of the material

    TetrahedronData<sofa::helper::vector<TetrahedronRestInformation> > m_tetrahedronInfo; ///< Internal tetrahedron data
    EdgeData<sofa::helper::vector<EdgeInformation> > m_edgeInfo; ///< Internal edge data

public:

    void setMaterialName(const string name) {
        d_materialName.setValue(name);
    }
    void setparameter(const vector<Real> param) {
        d_parameterSet.setValue(param);
    }
    void setdirection(const vector<Coord> direction) {
        d_anisotropySet.setValue(direction);
    }

    class SOFA_MISC_FEM_API TetrahedronHandler : public TopologyDataHandler<Tetrahedron,sofa::helper::vector<TetrahedronRestInformation> >
    {
    public:
      typedef typename TetrahedronNonlinearElasticityFEMForceField<DataTypes>::TetrahedronRestInformation TetrahedronRestInformation;
      TetrahedronHandler(TetrahedronNonlinearElasticityFEMForceField<DataTypes>* ff,
                         TetrahedronData<sofa::helper::vector<TetrahedronRestInformation> >* data )
        :TopologyDataHandler<Tetrahedron,sofa::helper::vector<TetrahedronRestInformation> >(data)
        ,ff(ff)
      {

      }

      void applyCreateFunction(unsigned int, TetrahedronRestInformation &t, const Tetrahedron &,
                               const sofa::helper::vector<unsigned int> &, const sofa::helper::vector<double> &);

    protected:
      TetrahedronNonlinearElasticityFEMForceField<DataTypes>* ff;
    };

protected:
   TetrahedronNonlinearElasticityFEMForceField();

   virtual   ~TetrahedronNonlinearElasticityFEMForceField();
public:

  //  virtual void parse(core::objectmodel::BaseObjectDescription* arg);

    void init() override;

    void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v) override;
    void addDForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& d_df, const DataVecDeriv& d_dx) override;
    SReal getPotentialEnergy(const core::MechanicalParams*, const DataVecCoord&) const override;
    void addKToMatrix(sofa::defaulttype::BaseMatrix *mat, SReal k, unsigned int &offset) override;

    void draw(const core::visual::VisualParams* vparams) override;

    Mat<3,3,double> getF( int tetrahedronIndex); // deformation Grad at the index tetrahedronIndex


  protected:

    /// the array that describes the complete material energy and its derivatives

    fem::NonlinearMaterial<DataTypes> *m_myMaterialModel;
    TetrahedronHandler* m_tetrahedronHandler;

    void testDerivatives();
    void saveMesh( const char *filename );

    void updateTangentMatrix();
};

using sofa::defaulttype::Vec3dTypes;
using sofa::defaulttype::Vec3fTypes;

#if  !defined(SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONHYPERELASTICITYFEMFORCEFIELD_CPP)

extern template class SOFA_MISC_FEM_API TetrahedronNonlinearElasticityFEMForceField<Vec3Types>;


#endif //  !defined(SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONHYPERELASTICITYFEMFORCEFIELD_CPP)

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
