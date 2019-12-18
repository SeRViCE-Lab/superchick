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
#include <IABPlugin/ForceFields/include/NonlinearElasticMaterial.h>
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

// Constructor of the class IsochoricForceField
// initializing data with their default value (here d_inputForTheUser=20)
template<typename DataTypes>
IsochoricForceField<DataTypes>::IsochoricForceField()
    : m_topology(0)
    , m_initialPoints(0)
    , m_updateMatrix(true)
    , m_meshSaved( false)
    , d_stiffnessMatrixRegularizationWeight(initData(&d_stiffnessMatrixRegularizationWeight, (bool)false,"matrixRegularization","Regularization of the Stiffness Matrix (between true or false)"))
    , d_materialName(initData(&d_materialName,std::string("MooneyRivlinIncompressible"),"materialName","the name of the material to be used"))
    , d_parameterSet(initData(&d_parameterSet,"ParameterSet","The global parameters specifying the material"))
    , d_anisotropySet(initData(&d_anisotropySet,"AnisotropyDirections","The global directions of anisotropy of the material"))
    , m_tetrahedronInfo(initData(&m_tetrahedronInfo, "tetrahedronInfo", "Internal tetrahedron data"))
    , m_edgeInfo(initData(&m_edgeInfo, "edgeInfo", "Internal edge data"))
    , m_tetrahedronHandler(NULL)
    /*
    indices(initData(&indices, "indices", "index of nodes controlled by the isochoric deformations")),
    d_Ri(initData(&d_Ri, Real(10.0), "Ri", "internal radius in the reference configuration")),
    d_Ro(initData(&d_Ro, Real(15.0), "Ro", "external radius in the reference configuration")),
    d_ri(initData(&d_ri, Real(13.0), "ri", "internal radius in the current configuration")),
    // d_ro(initData(&d_ro, Real(25), "ro", "external radius in the current configuration")),
    d_C1(initData(&d_C1, Real(1e3), "C1", "material elasticity of the internal IAB wall")),
    d_C2(initData(&d_C2, Real(1e3), "C2", "material elasticity of the outer IAB wall")),
    d_mode(initData(&d_mode, "expand", "mode", "mode of deformation: <expansion> or <compression>")),
    counter(0)
    */
{
  m_tetrahedronHandler = new TetrahedronHandler(this,&m_tetrahedronInfo);
  //default Constructor
  // init();
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::TetrahedronHandler::applyCreateFunction(unsigned int tetrahedronIndex,
                                                                            TetrahedronRestInformation &tinfo,
                                                                            const Tetrahedron &,
                                                                            const sofa::helper::vector<unsigned int> &,
                                                                            const sofa::helper::vector<double> &)
                                                                            const sofa::helper::vector<double> &)
{
  if (ff) {
      const vector< Tetrahedron > &tetrahedronArray=ff->m_topology->getTetrahedra() ;
      const std::vector< Edge> &edgeArray=ff->m_topology->getEdges() ;
      unsigned int j;
//      int k;
      typename DataTypes::Real volume;
      typename DataTypes::Coord point[4];
      const typename DataTypes::VecCoord restPosition = ff->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

      ///describe the indices of the 4 tetrahedron vertices
      const Tetrahedron &t= tetrahedronArray[tetrahedronIndex];
      BaseMeshTopology::EdgesInTetrahedron te=ff->m_topology->getEdgesInTetrahedron(tetrahedronIndex);

      // store the point position
      for(j=0;j<4;++j)
          point[j]=(restPosition)[t[j]];
      /// compute 6 times the rest volume
      volume=dot(cross(point[2]-point[0],point[3]-point[0]),point[1]-point[0]);
      /// store the rest volume
      tinfo.m_volScale =(Real)(1.0/volume);
      tinfo.m_restVolume = std::fabs(volume/6);
      // store shape vectors at the rest configuration
      for(j=0;j<4;++j) {
          if (!(j%2))
              tinfo.m_shapeVector[j]=-cross(point[(j+2)%4] - point[(j+1)%4],point[(j+3)%4] - point[(j+1)%4])/ volume;
          else
              tinfo.m_shapeVector[j]=cross(point[(j+2)%4] - point[(j+1)%4],point[(j+3)%4] - point[(j+1)%4])/ volume;;
      }


      for(j=0;j<6;++j) {
          Edge e=ff->m_topology->getLocalEdgesInTetrahedron(j);
          int k=e[0];
          //int l=e[1];
          if (edgeArray[te[j]][0]!=t[k]) {
              k=e[1];
              //l=e[0];
          }
      }


  }//end if(ff)

}


template<typename DataTypes>
IsochoricForceField<DataTypes>::~IsochoricForceField()
{
  if(m_tetrahedronHandler)
    delete m_tetrahedronHandler;
}


template<typename DataTypes>
void IsochoricForceField<DataTypes>::init()
{
    if (this->f_printLog.getValue())
        msg_info() << "initializing TetrahedronNonlinearElasticityFEMForceField";

    this->Inherited::init();

    /** parse the parameter set */
    SetParameterArray paramSet=d_parameterSet.getValue();
    if (paramSet.size()>0) {
            globalParameters.parameterArray.resize(paramSet.size());
            copy(paramSet.begin(), paramSet.end(),globalParameters.parameterArray.begin());
    }
    /** parse the anisotropy Direction set */
    SetAnisotropyDirectionArray anisotropySet=d_anisotropySet.getValue();
    if (anisotropySet.size()>0) {
            globalParameters.anisotropyDirection.resize(anisotropySet.size());
            copy(anisotropySet.begin(), anisotropySet.end(),globalParameters.anisotropyDirection.begin());
    }

    m_topology = this->getContext()->getMeshTopology();


    /** parse the input material name */
    string material = d_materialName.getValue();
    if (material=="MooneyRivlinIncompressible")
    {
        sofa::component::fem::NonlinearElastic<DataTypes> *NonlinearElasticMaterial = new sofa::component::fem::NonlinearElastic<DataTypes>;
        m_MRIncompMatlModel = NonlinearElasticMaterial;
        if (this->f_printLog.getValue())
            msg_info()<<"The model is "<<material;
    }
    else
    {
        msg_error() << "material name " << material << " is not valid (should be ArrudaBoyce, StVenantKirchhoff, MooneyRivlin, VerondaWestman, Costa or Ogden)";
    }


    if (!m_topology->getNbTetrahedra())
    {
        msg_error() << "ERROR(TetrahedronNonlinearElasticityFEMForceField): object must have a Tetrahedral Set Topology.\n";
        return;
    }

    helper::vector<typename TetrahedronNonlinearElasticityFEMForceField<DataTypes>::TetrahedronRestInformation>& tetrahedronInf = *(m_tetrahedronInfo.beginEdit());

    /// prepare to store info in the triangle array
    tetrahedronInf.resize(m_topology->getNbTetrahedra());

    helper::vector<typename TetrahedronNonlinearElasticityFEMForceField<DataTypes>::EdgeInformation>& edgeInf = *(m_edgeInfo.beginEdit());

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
    for (Topology::TetrahedronID i=0;i<m_topology->getNbTetrahedra();++i)
    {
        m_tetrahedronHandler->applyCreateFunction(i, tetrahedronInf[i],
                                                m_topology->getTetrahedron(i),  (const vector< unsigned int > )0,
                                                (const vector< double >)0);
    }

    /// set the call back function upon creation of a tetrahedron
    m_tetrahedronInfo.createTopologicalEngine(m_topology,m_tetrahedronHandler);
    m_tetrahedronInfo.registerTopologicalData();

    m_tetrahedronInfo.endEdit();
    //testDerivatives();

}
/* Old init() impl
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
}
*/

template<typename DataTypes>
void IsochoricForceField<DataTypes>::reinit()
{
  // not yet implemented
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                             DataVecDeriv& f, const DataVecCoord& x,
                                             const DataVecDeriv& v)
/*
{
    if(!mState)
    {
      msg_info("IsochoricForceField") << "No Mechanical State found, no force will be computed..." << "\n";
        return;
    }
    // Compute the forces f from the current DOFs p; here i am using the derived stress from eq 25v in paper 1
    helper::WriteAccessor< DataVecDeriv > f1 = f;
    helper::ReadAccessor< DataVecCoord >  p1 = x;
    helper::ReadAccessor< DataVecDeriv >  v1 = v;

    f1.resize(p1.size());

    SOFA_UNUSED(f);
    SOFA_UNUSED(x);
    SOFA_UNUSED(v);

    // calculate the stress needed to go from a reference configuartion to a current configuration
    radial_stress_r2c<double> sigma_r2c(m_Ri, m_Ro, m_ri, m_C1, m_C2);
    // auto stress_rr = integrator<double, radial_stress_r2c<double>>(m_ri, m_ro, abstol, reltol, sigma_r2c);
    auto stress_rr = integrator<double, radial_stress_r2c<double>>(m_ri,
                            m_ro,
                            abstol,
                            reltol,
                            radial_stress_r2c<double>(m_Ri, m_Ro, m_ri, m_C1, m_C2));

    // calculate the pressure needed to go from a reference configuartion to a current configuration
    // pressure_r2c<double> P_r2c(m_Ri, m_Ro, m_ri, m_C1, m_C2);
    // auto pressure = integrator<double, pressure_r2c<double>>(m_ri, m_ro, abstol, reltol, P_r2c);
    auto pressure = -1*stress_rr;
    msg_info("IsochoricForceField") << "Calculated normal stress based on given parameters:" <<
                    "\n\tC1: " << m_C1 << " C2: " << m_C2 << " Ri: " <<  m_Ri  << " Ro: " <<  m_Ro <<
                    "\n\tri: " <<  m_ri  <<  " ro: " << m_ro << " stress_rr: " << stress_rr <<
                    "\n\tP: " << pressure << "\n";
  }
*/
template<typename DataTypes>
void IsochoricForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams,
                                              DataVecDeriv& d_f , const DataVecDeriv& d_x,
                                              const DataVecDeriv& /* d_v */)
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

    helper::vector<TetrahedronRestInformation>& tetrahedronInf = *(m_tetrahedronInfo.beginEdit());

    TetrahedronRestInformation *tetInfo;

    assert(this->mstate);

    Coord dp[3],x0,sv;

    // opportunity for improvement using my formulation here
    for(i=0; i<nbTetrahedra; i++ )
    {
        tetInfo=&tetrahedronInf[i];
        const Tetrahedron &ta= m_topology->getTetrahedron(i);

        x0=x[ta[0]];

        // compute the deformation gradient
        // deformation gradient = sum of tensor product between vertex position and shape vector
        // optimize by using displacement with first vertex
        dp[0]=x[ta[1]]-x0;
        sv=tetInfo->m_shapeVector[1];
        for (k=0;k<3;++k) {
                for (l=0;l<3;++l) {
                        tetInfo->m_deformationGradient[k][l]=dp[0][k]*sv[l];
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

        /// compute the right Cauchy-Green deformation matrix: F^TF
        for (k=0;k<3;++k) {
            for (l=k;l<3;++l) {
                tetInfo->rightCauchy(k,l)=\
               (tetInfo->m_deformationGradient(0,k)*tetInfo->m_deformationGradient(0,l)+
                tetInfo->m_deformationGradient(1,k)*tetInfo->m_deformationGradient(1,l)+
                tetInfo->m_deformationGradient(2,k)*tetInfo->m_deformationGradient(2,l));
            }
        }

        // don't know if I need this
        if(globalParameters.anisotropyDirection.size()>0)
        {
            tetInfo->m_fiberDirection=globalParameters.anisotropyDirection[0];
            Coord vectCa=tetInfo->rightCauchy*tetInfo->m_fiberDirection;
            Real aDotCDota=dot(tetInfo->m_fiberDirection,vectCa);
            tetInfo->lambda=(Real)sqrt(aDotCDota);
        }
        Coord areaVec = cross( dp[1], dp[2] );

        tetInfo->J = dot( areaVec, dp[0] ) * tetInfo->m_volScale;
        tetInfo->trC = (Real)( tetInfo->rightCauchy(0,0) + tetInfo->rightCauchy(1,1) + tetInfo->rightCauchy(2,2));
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
