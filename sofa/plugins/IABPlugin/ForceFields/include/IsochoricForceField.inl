/*
*  Ripped off sofa/modules/SofaMiscFem/TetrahedronHyperelasticityFEMForceField.inl
*
Author: Lekan Ogunmolux, December 18, 2019
*/
#ifndef SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_INL

#include <fstream> // for reading the file
#include <cassert>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <sofa/helper/rmath.h>
#include <sofa/helper/system/gl.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/system/config.h>
#include <sofa/defaulttype/RGBAColor.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/ForceField.inl>
#include <SofaBaseTopology/TopologyData.inl>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "IABPlugin/ForceFields/include/integrand.inl"  // will help with our integrations
#include "IABPlugin/ForceFields/include/NonlinearElasticMaterial.h"
#include "IABPlugin/ForceFields/include/MooneyRivlinIncompressible.h"
#include "IABPlugin/ForceFields/include/IsochoricForceField.h"

using namespace boost::numeric::odeint;

// see SoftRobots/model/SurfacePressureModel
namespace sofa
{

namespace component
{

namespace forcefield
{
  using namespace sofa::defaulttype;
  using namespace	sofa::component::topology;
  using namespace core::topology;

template<typename DataTypes>
void IsochoricForceField<DataTypes>::SphericalPolarHandler::applyCreateFunction(unsigned int sphereIndex,
                                                                            SphericalPolarRestInformation &sinfo,
                                                                            const Sphere &,
                                                                            const sofa::helper::vector<unsigned int> &,
                                                                            const sofa::helper::vector<double> &)
{
  if (ff) {
      const vector< Sphere > &sphereArray=ff->m_topology->getTriangles() ;
      const std::vector< Edge> &edgeArray=ff->m_topology->getEdges() ;
      unsigned int j;
//      int k;
      typename DataTypes::Real volume;
      typename DataTypes::Coord point[3];
      const typename DataTypes::VecCoord restPosition = ff->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

      ///describe the indices of the 3 spherical coordinates at a point
      const Sphere &t= sphereArray[sphereIndex];
      BaseMeshTopology::EdgesInTriangle te=ff->m_topology->getEdgesInTriangle(sphereIndex);

      // store the point position
      for(j=0;j<3;++j) //was j<4
          point[j]=(restPosition)[t[j]];
      /// compute 6 times the rest volume
      // volume=dot(cross(point[2]-point[0],point[3]-point[0]),point[1]-point[0]);
      m_radialVector[0] = point[0]; //m_r * std::cos(m_theta) * std::sin(m_phi);
      m_radialVector[1] = point[1]; //m_r * std::sin(m_theta) * std::sin(m_phi);
      m_radialVector[2] = point[2]; //m_r * std::cos(m_theta);

      volume=(2/3)*M_Pi*std::norm(m_radialVector);
      /// store the rest volume
      sinfo.m_restVolume = std::fabs(volume);

      // for(j=0;j<6;++j) {
      //     Edge e=ff->m_topology->getLocalEdgesInTetrahedron(j);
      //     int k=e[0];
      //     //int l=e[1];
      //     if (edgeArray[te[j]][0]!=t[k]) {
      //         k=e[1];
      //         //l=e[0];
      //     }
      // }


  }//end if(ff)

}

// Constructor of the class IsochoricForceField
// initializing data with their default value (here d_inputForTheUser=20)
template<typename DataTypes>
IsochoricForceField<DataTypes>::IsochoricForceField()
    : m_topology(0)
    , m_initialPoints(0)
    , m_updateMatrix(true)
    , m_meshSaved(false)
    , d_stiffnessMatrixRegularizationWeight(initData(&d_stiffnessMatrixRegularizationWeight, (bool)false,"matrixRegularization","Regularization of the Stiffness Matrix (between true or false)"))
    , d_materialName(initData(&d_materialName,std::string("MooneyRivlinIncompressible"),"materialName","the name of the material to be used"))
    , d_parameterSet(initData(&d_parameterSet,"ParameterSet","The global parameters specifying the material"))
    , m_sphericalPolarInfo(initData(&m_sphericalPolarInfo, "m_sphericalPolarInfo", "Internal spherical data"))
    , m_edgeInfo(initData(&m_edgeInfo, "edgeInfo", "Internal edge data"))
    , m_sphericalPolarHandler(nullptr)
{
  m_sphericalPolarHandler = new SphericalPolarHandler(this, &m_sphericalPolarInfo);
}

template<typename DataTypes>
IsochoricForceField<DataTypes>::~IsochoricForceField()
{
  if (m_sphericalPolarHandler)
    delete m_sphericalPolarHandler;
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::init()
{
    if (this->f_printLog.getValue())
        msg_info() << "initializing IsochoricForceField";

    this->Inherited::init();

    /** parse the parameter set */
    SetParameterArray paramSet=d_parameterSet.getValue();
    if (paramSet.size()>0) {
            globalParameters.parameterArray.resize(paramSet.size());
            copy(paramSet.begin(), paramSet.end(),globalParameters.parameterArray.begin());
    }

    m_topology = this->getContext()->getMeshTopology();


    /** parse the input material name */
    string material = d_materialName.getValue();
    if (material=="MooneyRivlinIncompressible")
    {
        fem::NonlinearElasticMaterial<DataTypes> *NonlinearElasticMaterialPtr = new sofa::component::fem::NonlinearElasticMaterial<DataTypes>;
        m_MRIncompMatlModel = NonlinearElasticMaterialPtr; // InCompressible Material Model of Mooney and Rivlin
        if (this->f_printLog.getValue())
            msg_info()<<"The model is "<<material;
    }
    else
    {
        msg_error() << "material name " << material << " is not valid (should be ArrudaBoyce, StVenantKirchhoff, MooneyRivlin, VerondaWestman, Costa or Ogden)";
    }


    if (!m_topology->getNbTriangles())
    {
        msg_error() << "ERROR(IsochoricForceField): object must have a Triangle Set Topology.\n";
        return;
    }

    helper::vector<typename IsochoricForceField<DataTypes>::SphericalPolarRestInformation>& sphereInf = *(m_sphericalPolarInfo.beginEdit());

    /// prepare to store info in the triangle array
    sphereInf.resize(m_topology->getNbTriangles());

    helper::vector<typename IsochoricForceField<DataTypes>::EdgeInformation>& edgeInf = *(m_edgeInfo.beginEdit());

    edgeInf.resize(m_topology->getNbEdges());
    m_edgeInfo.createTopologicalEngine(m_topology);

    m_edgeInfo.registerTopologicalData();

    m_edgeInfo.endEdit();

    // get restPosition
    if (m_initialPoints.size() == 0)
    {
    const VecCoord& p = this->mstate->read(core::ConstVecCoordId::restPosition())->getValue();
            m_initialPoints=p;
    }

    /// initialize the data structure associated with each tetrahedron
    for (Topology::TriangleID triID=0; triID<m_topology->getNbTriangles(); ++triID)
    {
        m_sphericalPolarHandler->applyCreateFunction(triID, sphereInf[triID],
                                                m_topology->getTriangles(triID),
                                                (const vector< unsigned int > )0,
                                                (const vector< double >)0);
    }

    /// set the call back function upon creation of a tetrahedron
    m_sphericalPolarInfo.createTopologicalEngine(m_topology,m_sphericalPolarHandler);
    m_sphericalPolarInfo.registerTopologicalData();

    m_sphericalPolarInfo.endEdit();
    //testDerivatives();
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::reinit()
{
  // not yet implemented
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                             DataVecDeriv& d_f, const DataVecCoord& d_x,
                                             const DataVecDeriv& /*d_v*/)
{
    VecDeriv& f = *d_f.beginEdit();
    const VecCoord& x = d_x.getValue();


    const bool printLog = this->f_printLog.getValue();
    if (printLog && !m_meshSaved)
    {
        saveMesh( "/opt/sofa-result.stl" );
        printf( "Mesh saved.\n" );
        m_meshSaved = true;
    }
    unsigned int i=0,j=0,k=0,l=0;
    unsigned int nbTriangles=m_topology->getNbTriangles();

    helper::vector<SphericalPolarRestInformation>& sphereInf = *(m_sphericalPolarInfo.beginEdit());

    SphericalPolarRestInformation *sphInfo;

    assert(this->mstate);

    Coord dp[3],x0,sv;


    for(i=0; i<nbTriangles; i++ )
    {
        sphInfo=&sphereInf[i];
        const Sphere &ta= m_topology->getTriangle(i);

        x0=x[ta[0]];

        // compute the deformation gradient
        // deformation gradient = sum of tensor product between vertex position and shape vector
        // optimize by using displacement with first vertex
        dp[0]=x[ta[1]]-x0;
        sv=sphInfo->m_shapeVector[1];
        for (k=0;k<3;++k) {
                for (l=0;l<3;++l) {
                        sphInfo->m_deformationGradient[k][l]=dp[0][k]*sv[l];
                }
        }
        for (j=1;j<3;++j) {
                dp[j]=x[ta[j+1]]-x0;
                sv=tetInfo->m_shapeVector[j+1];
                for (k=0;k<3;++k) {
                        for (l=0;l<3;++l) {
                                tetInfo->m_deformationGradient[k][l]+=dp[j][k]*sv[l];
                        }
                }
        }

        /// compute the right Cauchy-Green deformation matrix
        // for (k=0;k<3;++k) {
        //     for (l=k;l<3;++l) {
        //         tetInfo->deformationTensor(k,l)=(tetInfo->m_deformationGradient(0,k)*tetInfo->m_deformationGradient(0,l)+
        //         tetInfo->m_deformationGradient(1,k)*tetInfo->m_deformationGradient(1,l)+
        //         tetInfo->m_deformationGradient(2,k)*tetInfo->m_deformationGradient(2,l));
        //     }
        // }
        // myne right cauchy-green tensor
        for (k=0;k<3;++k) {
            for (l=k;l<3;++l) {
                tetInfo->rightCauchy(k,l)=(tetInfo->m_deformationGradient(0,k)*tetInfo->m_deformationGradient(0,l)+
                tetInfo->m_deformationGradient(1,k)*tetInfo->m_deformationGradient(1,l)+
                tetInfo->m_deformationGradient(2,k)*tetInfo->m_deformationGradient(2,l));
            }
        }

        Coord areaVec = cross( dp[1], dp[2] );

        tetInfo->J = dot( areaVec, dp[0] ) * tetInfo->m_volScale;
        tetInfo->trC = (Real)( tetInfo->deformationTensor(0,0) + tetInfo->deformationTensor(1,1) + tetInfo->deformationTensor(2,2));
        tetInfo->m_SPKTensorGeneral.clear();
        m_MRIncompMatlModel->deriveSPKTensor(tetInfo,globalParameters,tetInfo->m_SPKTensorGeneral);
        for(l=0;l<4;++l)
        {
            f[ta[l]]-=tetInfo->m_deformationGradient*(tetInfo->m_SPKTensorGeneral*tetInfo->m_shapeVector[l])*tetInfo->m_restVolume;
        }
    }


    /// indicates that the next call to addDForce will need to update the stiffness matrix
    m_updateMatrix=true;
    m_tetrahedronInfo.endEdit();

    d_f.endEdit();
}


template<typename DataTypes>
void IsochoricForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams,
                                              DataVecDeriv& d_f , const DataVecDeriv& d_x)
{
    // Compute the force derivative d_df from the current, which will be multiplied with the field d_dx
    VecDeriv& f = *d_f.beginEdit();
    const VecCoord& x = d_x.getValue();


    const bool printLog = this->f_printLog.getValue();
    if (printLog && !m_meshSaved)
    {
        saveMesh( "/opt/sofa-result.stl" );
        printf( "Mesh saved.\n" );
        m_meshSaved = true;
    }
    unsigned int i=0,j=0,k=0,l=0;
    unsigned int nbTetrahedra=m_topology->getNbTetrahedra();

    helper::vector<SphericalPolarRestInformation>& sphereInf = *(m_tetrahedronInfo.beginEdit());

    SphericalPolarRestInformation *sphInfo;

    assert(this->mstate);

    Coord dp[3],x0,sv;

    // opportunity for improvement using my formulation here
    for(i=0; i<nbTetrahedra; i++ )
    {
        sphInfo=&tetrahedronInf[i];
        const Tetrahedron &ta= m_topology->getTetrahedron(i);

        x0=x[ta[0]];

        // compute the deformation gradient
        // deformation gradient = sum of tensor product between vertex position and shape vector
        // optimize by using displacement with first vertex
        dp[0]=x[ta[1]]-x0;
        sv=sphInfo->m_shapeVector[1];
        for (k=0;k<3;++k) {
                for (l=0;l<3;++l) {
                        sphInfo->m_deformationGradient[k][l]=dp[0][k]*sv[l];
                }
        }
        for (j=1;j<3;++j) {
                dp[j]=x[ta[j+1]]-x0;
                sv=sphInfo->m_shapeVector[j+1];
                for (k=0;k<3;++k) {
                        for (l=0;l<3;++l) {
                                sphInfo->m_deformationGradient[k][l]+=dp[j][k]*sv[l];
                        }
                }
        }

        /// compute the right Cauchy-Green deformation matrix: F^TF
        for (k=0;k<3;++k) {
            for (l=k;l<3;++l) {
                sphInfo->rightCauchy(k,l)=\
               (sphInfo->m_deformationGradient(0,k)*sphInfo->m_deformationGradient(0,l)+
                sphInfo->m_deformationGradient(1,k)*sphInfo->m_deformationGradient(1,l)+
                sphInfo->m_deformationGradient(2,k)*sphInfo->m_deformationGradient(2,l));
            }
        }

        Coord areaVec = cross( dp[1], dp[2] );

        sphInfo->J = dot( areaVec, dp[0] ) * sphInfo->m_volScale;
        sphInfo->trC = (Real)( sphInfo->rightCauchy(0,0) + sphInfo->rightCauchy(1,1) + sphInfo->rightCauchy(2,2));
        sphInfo->m_SPKTensorGeneral.clear();
        m_MRIncompMatlModel->deriveSPKTensor(sphInfo,globalParameters,sphInfo->m_SPKTensorGeneral);
        for(l=0;l<4;++l)
        {
            f[ta[l]]-=sphInfo->m_deformationGradient*(sphInfo->m_SPKTensorGeneral*tetInfo->m_shapeVector[l])*tetInfo->m_restVolume;
        }
    }


    /// indicates that the next call to addDForce will need to update the stiffness matrix
    m_updateMatrix=true;
    m_tetrahedronInfo.endEdit();

    d_f.endEdit();
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

template<class DataTypes>
void IsochoricForceField<DataTypes>::saveMesh( const char *filename )
{
    VecCoord pos( this->mstate->read(core::ConstVecCoordId::position())->getValue());
    core::topology::BaseMeshTopology::SeqTriangles triangles = m_topology->getTriangles();
    FILE *file = fopen( filename, "wb" );

    if (!file) return;

    // write header
    char header[81];

    size_t errResult;
    errResult = fwrite( (void*)&(header[0]),1, 80, file );
    unsigned int numTriangles = triangles.size();
    errResult = fwrite( &numTriangles, 4, 1, file );
    // write poly data
    float vertex[3][3];
    float normal[3] = { 1,0,0 };
    short stlSeperator = 0;

    for (unsigned int triangleId=0; triangleId<triangles.size(); triangleId++)
    {
        if (m_topology->getTetrahedraAroundTriangle( triangleId ).size()==1)
        {
            // surface triangle, save it
            unsigned int p0 = m_topology->getTriangle( triangleId )[0];
            unsigned int p1 = m_topology->getTriangle( triangleId )[1];
            unsigned int p2 = m_topology->getTriangle( triangleId )[2];
            for (int d=0; d<3; d++)
            {
                    vertex[0][d] = (float)pos[p0][d];
                    vertex[1][d] = (float)pos[p1][d];
                    vertex[2][d] = (float)pos[p2][d];
            }
            errResult = fwrite( (void*)&(normal[0]), sizeof(float), 3, file );
            errResult = fwrite( (void*)&(vertex[0][0]), sizeof(float), 9, file );
            errResult = fwrite( (void*)&(stlSeperator), 2, 1, file );
        }
    }
    errResult -= errResult; // ugly trick to avoid warnings

	fclose( file );
}

} // namespace forcefield
} // namespace component
} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_ISOCHORICFORCEFIELD_INL
