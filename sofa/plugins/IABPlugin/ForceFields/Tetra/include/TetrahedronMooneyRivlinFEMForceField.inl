/*
*  Ripped off sofa/modules/SofaMiscFem/TetrahedronMooneyRivlinFEMForceField.inl
*
*  Author: Lekan Molux, December 18, 2019
*/
#ifndef SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_INL

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
#include "IABPlugin/ForceFields/Tetra/include/TetrahedronMooneyRivlinFEMForceField.h"


namespace sofa
{
namespace component
{
namespace forcefield
{
using namespace sofa::defaulttype;
using namespace	sofa::component::topology;
using namespace core::topology;
using namespace sofa::helper; // for M_PI: see rmath.h

template<typename DataTypes >
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::TetrahedronHandler::applyCreateFunction(unsigned int tetrahedronIndex,
                                                                                              TetrahedronRestInformation &tinfo,
                                                                                              const Tetrahedron &,
                                                                                              const sofa::helper::vector<unsigned int> &,
                                                                                              const sofa::helper::vector<double> &)
{
  if (ff)
  {
      const vector< Tetrahedron > &tetrahedronArray=ff->m_topology->getTetrahedra() ;
      const std::vector< Edge> &edgeArray=ff->m_topology->getEdges() ;
      unsigned int j;

      typename DataTypes::Real volume;
      typename DataTypes::Coord point[4];
      const typename DataTypes::VecCoord restPosition = ff->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

      ///describe the indices of the 4 tetrahedron vertices
      const Tetrahedron &t= tetrahedronArray[tetrahedronIndex];
      BaseMeshTopology::EdgesInTetrahedron te=ff->m_topology->getEdgesInTetrahedron(tetrahedronIndex);
      tinfo.m_sPolarVecLag.resize(4);

      // store the vertex positions of the tetrahedron
      for(j=0;j<4;++j)
      {
        point[j]=(restPosition)[t[j]];
        // for each point in point[j], find the spherical polar coordinate equivalent assuming tetrahedron is platonic
        tinfo.m_sPolarVecLag[j].m_R =  rsqrt(SQR(point[j][0])+SQR(point[j][1])+SQR(point[j][2])); // r
        tinfo.m_sPolarVecLag[j].m_Theta =  std::atan(point[j][1]/point[j][0]);    // theta = arctan(y/x)
        tinfo.m_sPolarVecLag[j].m_Phi =  std::acos(point[j][2]/tinfo.m_sPolarVecLag[j].m_R);    // theta = arctan(z/r)
        tinfo.m_sPolarVecLag[j].m_RadialVector = {tinfo.m_sPolarVecLag[j].m_R, tinfo.m_sPolarVecLag[j].m_Theta, tinfo.m_sPolarVecLag[j].m_Phi};

        // gamma in lagrangean axis is fixed
        tinfo.m_sPolarVecLag[j].Gamma = deg2rad(0);
      }
      volume=dot(cross(point[2]-point[0],point[3]-point[0]),point[1]-point[0]);
      /// store the rest volume
      tinfo.m_volScale =(Real)(1.0/volume);
      tinfo.m_restVolume = fabs(volume/6);

      // store shape vectors at the rest configuration
      for(j=0;j<4;++j)
      {
          if (!(j%2))
              tinfo.m_shapeVector[j]=-cross(point[(j+2)%4] - point[(j+1)%4],point[(j+3)%4] - point[(j+1)%4])/ volume;
          else
              tinfo.m_shapeVector[j]=cross(point[(j+2)%4] - point[(j+1)%4],point[(j+3)%4] - point[(j+1)%4])/ volume;;
      }

      for(j=0;j<6;++j)
      {
          Edge e=ff->m_topology->getLocalEdgesInTetrahedron(j);
          int k=e[0];
          if (edgeArray[te[j]][0]!=t[k]) {
              k=e[1];
          }
      }
  }
}

template <typename DataTypes>
TetrahedronMooneyRivlinFEMForceField<DataTypes>::TetrahedronMooneyRivlinFEMForceField()
    : m_topology(0)
    , m_initialPoints(0)
    , m_updateMatrix(true)
    , m_meshSaved(false)
    , d_stiffnessMatrixRegularizationWeight(initData(&d_stiffnessMatrixRegularizationWeight, (bool)false,"matrixRegularization","Regularization of the Stiffness Matrix (between true or false)"))
    , d_materialName(initData(&d_materialName,std::string("MooneyRivlinIncompressible"),"materialName","the name of the material to be used"))
    , d_parameterSet(initData(&d_parameterSet,"ParameterSet","The global parameters specifying the material"))
    , d_anisotropySet(initData(&d_anisotropySet,"AnisotropyDirections","The global directions of anisotropy of the material"))
    , m_tetrahedronInfo(initData(&m_tetrahedronInfo, "tetrahedronInfo", "Internal tetrahedron data"))
    , m_edgeInfo(initData(&m_edgeInfo, "edgeInfo", "Internal edge data"))
    , f_poisson(initData(&f_poisson,helper::vector<Real>(1,static_cast<Real>(0.45)),"poissonRatio","Poisson ratio in Hooke's law (vector)"))
    , f_young(initData(&f_young,helper::vector<Real>(1,static_cast<Real>(1000.0)),"youngModulus","Young modulus in Hooke's law (vector)"))
    , f_damping(initData(&f_damping,(Real)0.,"damping","Ratio damping/stiffness"))
    , m_tetrahedronHandler(nullptr)
{
    m_tetrahedronHandler = new TetrahedronHandler(this,&m_tetrahedronInfo);
    f_poisson.setRequired(true);
    f_young.setRequired(true);
}

template <typename DataTypes>
TetrahedronMooneyRivlinFEMForceField<DataTypes>::~TetrahedronMooneyRivlinFEMForceField()
{
    if(m_tetrahedronHandler) delete m_tetrahedronHandler;
}

template <typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::init()
{
    // if (this->f_printLog.getValue())
    msg_info() << "initializing TetrahedronMooneyRivlinFEMForceField";

    this->Inherited::init();

    /** parse the parameter set */
    SetParameterArray paramSet=d_parameterSet.getValue();
    if (paramSet.size()>0)
    {
      globalParameters.parameterArray.resize(paramSet.size());
      copy(paramSet.begin(), paramSet.end(),globalParameters.parameterArray.begin());
    }
    /** parse the anisotropy Direction set */
    SetAnisotropyDirectionArray anisotropySet=d_anisotropySet.getValue();
    if (anisotropySet.size()>0)
    {
      globalParameters.anisotropyDirection.resize(anisotropySet.size());
      copy(anisotropySet.begin(), anisotropySet.end(),globalParameters.anisotropyDirection.begin());
    }

    m_topology = this->getContext()->getMeshTopology();

    /** parse the input material name */
    string material = d_materialName.getValue();
    if (material=="MooneyRivlinIncompressible")
    {
        fem::NonlinearElasticMaterial<DataTypes> *NonlinearElasticMaterialPtr = new sofa::component::fem::NonlinearElasticMaterial<DataTypes>;
        m_MRIncompMatlModel = NonlinearElasticMaterialPtr; // InCompressible Material Model of Mooney and Rivlin
        // if (this->f_printLog.getValue())
        msg_info()<<"The model is "<<material;
    }
    else
    {
        msg_error() << "material name " << material << " is not valid (should be MooneyRivlinIncompressible)";
    }


    if (!m_topology->getNbTetrahedra())
    {
        msg_error() << "ERROR(TetrahedronMooneyRivlinFEMForceField): object must have a Tetrahedral Set Topology.\n";
        return;
    }

    helper::vector<typename TetrahedronMooneyRivlinFEMForceField<DataTypes>::TetrahedronRestInformation>& tetrahedronInfVec = *(m_tetrahedronInfo.beginEdit());

    /// prepare to store info in the triangle array
    tetrahedronInfVec.resize(m_topology->getNbTetrahedra());

    helper::vector<typename TetrahedronMooneyRivlinFEMForceField<DataTypes>::EdgeInformation>& edgeInf = *(m_edgeInfo.beginEdit());

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
        m_tetrahedronHandler->applyCreateFunction(i, tetrahedronInfVec[i],
                                                m_topology->getTetrahedron(i),
                                                (const vector< unsigned int > )0,
                                                (const vector< double >)0);
        tetrahedronInfVec[i].m_C1 = paramSet[0];
        tetrahedronInfVec[i].m_C2 = paramSet[1];
    }

    /// set the call back function upon creation of a tetrahedron
    m_tetrahedronInfo.createTopologicalEngine(m_topology,m_tetrahedronHandler);
    m_tetrahedronInfo.registerTopologicalData();

    m_tetrahedronInfo.endEdit();
}

template <typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::reinit()
{ /*Nothing to do here, Scotty! */}

template <typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::addForce(const core::MechanicalParams* , DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& /* d_v */)
{
    VecDeriv& f = *d_f.beginEdit();
    const VecCoord& x = d_x.getValue();
    unsigned int nbTetrahedra=m_topology->getNbTetrahedra();

    helper::vector<TetrahedronRestInformation>& tetrahedronInfVec = *(m_tetrahedronInfo.beginEdit());

    TetrahedronRestInformation *tetInfo;

    assert(this->mstate);

    for(uint i=0; i<nbTetrahedra; i++ )
    {
        tetInfo=&tetrahedronInfVec[i];
        const Tetrahedron &ta= m_topology->getTetrahedron(i);

        tetInfo->m_sPolarVecEul.resize(4);
        // compute associated spherical points from the tetrahedron triangles
        for (uint j = 0; j < 4; ++j)
        {
          tetInfo->m_sPolarVecEul[j].m_r = std::sqrt(SQ(x[ta[j]][0]) + SQ(x[ta[j]][1]) + SQ(x[ta[j]][2])); // r
          tetInfo->m_sPolarVecEul[j].m_theta = std::atan(x[ta[j]][1]/x[ta[j]][0]);    // theta = arctan(y/x)
          tetInfo->m_sPolarVecEul[j].m_phi = std::acos(x[ta[j]][2]/tetInfo->m_sPolarVecEul[j].m_r);    // phi = arctan(z/r)
          tetInfo->m_sPolarVecEul[j].m_radialVector = {tetInfo->m_sPolarVecEul[j].m_r, tetInfo->m_sPolarVecEul[j].m_theta, tetInfo->m_sPolarVecEul[j].m_phi};

          // initialize associated tri components
          tetInfo->m_sPolarVecEul[j].m_ro = tetInfo->m_sPolarVecEul[j].m_r;

          tetInfo->m_sPolarVecEul[j].gamma = 0.25 * M_PI; // assume fiber angles are 45 degrees and constant
          // compute F, B, and C (defGrad, left and right Cauchy-Green Tensors)

          // initialize bottom left block of def grad to zeros
          for (uint k = 1; k < 3; ++k)  // rows 1 and 2
            for(uint l=0; l <2; ++l)  // cols 0 through 1
              tetInfo->m_sPolarVecEul[j].m_F[k][l] =  0;
          tetInfo->m_sPolarVecEul[j].m_F[0][0] = SQ(tetInfo->m_sPolarVecLag[j].m_R)/SQ(tetInfo->m_sPolarVecEul[j].m_r);
          tetInfo->m_sPolarVecEul[j].m_F[0][1] = -tetInfo->m_sPolarVecEul[j].m_phi/tetInfo->m_sPolarVecLag[j].m_R;
          tetInfo->m_sPolarVecEul[j].m_F[0][2] = (-tetInfo->m_sPolarVecEul[j].m_theta/tetInfo->m_sPolarVecLag[j].m_R)*\
                                                  cot(tetInfo->m_sPolarVecEul[j].m_phi);
          tetInfo->m_sPolarVecEul[j].m_F[1][1] = (tetInfo->m_sPolarVecEul[j].m_r/tetInfo->m_sPolarVecLag[j].m_R)+(1/(tetInfo->m_sPolarVecLag[j].m_R));
          tetInfo->m_sPolarVecEul[j].m_F[1][2] = -(tetInfo->m_sPolarVecEul[j].m_theta/tetInfo->m_sPolarVecLag[j].m_R)*\
                                cot(tetInfo->m_sPolarVecEul[j].m_phi);
          tetInfo->m_sPolarVecEul[j].m_F[2][2] = (tetInfo->m_sPolarVecEul[j].m_r/tetInfo->m_sPolarVecLag[j].m_R)+\
                                (tetInfo->m_sPolarVecEul[j].m_phi/tetInfo->m_sPolarVecLag[j].m_R)*cot(tetInfo->m_sPolarVecEul[j].m_phi)+\
                                (1/(tetInfo->m_sPolarVecLag[j].m_R*std::sin(tetInfo->m_sPolarVecLag[j].m_Phi)));
          tetInfo->m_sPolarVecEul[j].J = determinant(tetInfo->m_sPolarVecEul[j].m_F);

          // init all non-diagonal elements to zero
          for(int m=0; m < 3; ++m)
          {
            for(int n=0; n < 3; ++n)
            {
              if (m==n)
              {
                tetInfo->m_sPolarVecEul[j].m_C[m][n] = SQ(tetInfo->m_sPolarVecEul[j].m_F[m][n]); //*tetInfo->m_sPolarVecEul[j].m_F[m][n];
                tetInfo->m_sPolarVecEul[j].m_B[m][n] = SQ(tetInfo->m_sPolarVecEul[j].m_F[m][n]); //*tetInfo->m_sPolarVecEul[j].m_F[m][n];
              }
              else
              {
                tetInfo->m_sPolarVecEul[j].m_C[m][n] = 0;
                tetInfo->m_sPolarVecEul[j].m_B[m][n] = 0;
              } // end if
            } // inner for
          } // outer for
          // fiber direction in eulerian coord
          tetInfo->m_sPolarVecEul[j].m_fiberDirection = {tetInfo->m_sPolarVecEul[j].m_F[0][0]*rcos(tetInfo->m_sPolarVecEul[j].gamma), \
                                                         tetInfo->m_sPolarVecEul[j].m_F[1][1]*rsin(tetInfo->m_sPolarVecEul[j].gamma), 0};
        } // end tetra vertices for
      } // end all nbTetrahedra for i
    /// indicates that the next call to addDForce will need to update the stiffness matrix
    m_updateMatrix=false; //true;  // do not update edge info for now.
    m_tetrahedronInfo.endEdit();

    d_f.endEdit();
}

template <typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::updateTangentMatrix()
{
    unsigned int i=0,j=0,k=0,l=0;
    unsigned int nbEdges=m_topology->getNbEdges();
    msg_info("nbEdges ") << nbEdges ;
    const vector< Edge> &edgeArray=m_topology->getEdges() ;

    helper::vector<EdgeInformation>& edgeInf = *(m_edgeInfo.beginEdit());
    helper::vector<TetrahedronRestInformation>& tetrahedronInfVec = *(m_tetrahedronInfo.beginEdit());

    EdgeInformation *einfo;
    TetrahedronRestInformation *tetInfo;
    unsigned int nbTetrahedra=m_topology->getNbTetrahedra();
    const std::vector< Tetrahedron> &tetrahedronArray=m_topology->getTetrahedra() ;

    for(l=0; l<nbEdges; l++ )
      edgeInf[l].DfDx.clear();
    for(i=0; i<nbTetrahedra; i++ )
    {
        tetInfo=&tetrahedronInfVec[i];
        // avg all def grads for this tetrahedron
        Matrix3 df(0);   // explicit constructor from an element
        for (uint j = 0; j < 4; ++j)
        {
          for (uint k=0; k < 3; ++k)
          {
            for (uint l=0; l < 3; ++l)
            {
              df[k][l] += tetInfo->m_sPolarVecEul[j].m_F[k][l];
            }
          }
        }
        df /= 4;

        BaseMeshTopology::EdgesInTetrahedron te=m_topology->getEdgesInTetrahedron(i);

        /// describe the jth vertex index of triangle no i
        const Tetrahedron &ta= tetrahedronArray[i];
        for(j=0;j<6;j++)
        {
            einfo= &edgeInf[te[j]];
            Edge e=m_topology->getLocalEdgesInTetrahedron(j);

            k=e[0];
            l=e[1];
            if (edgeArray[te[j]][0]!=ta[k]) {
                k=e[1];
                l=e[0];
            }
            Matrix3 &edgeDfDx = einfo->DfDx;

            Coord svl=tetInfo->m_shapeVector[l];
            Coord svk=tetInfo->m_shapeVector[k];

            Matrix3  M, N;
            MatrixSym outputTensor;
            N.clear();
            vector<MatrixSym> inputTensor;
            inputTensor.resize(3);
            //	MatrixSym input1,input2,input3,outputTensor;
            for(int m=0; m<3;m++){
                for (int n=m;n<3;n++){
                    inputTensor[0](m,n)=svl[m]*df[0][n]+df[0][m]*svl[n];
                    inputTensor[1](m,n)=svl[m]*df[1][n]+df[1][m]*svl[n];
                    inputTensor[2](m,n)=svl[m]*df[2][n]+df[2][m]*svl[n];
                }
            }

            for(int m=0; m<3; m++){
                m_MRIncompMatlModel->applyElasticityTensor(tetInfo,globalParameters,inputTensor[m],outputTensor);
                Coord vectortemp=df*(outputTensor*svk);
                Matrix3 Nv;
                //Nv.clear();
                for(int u=0; u<3;u++){
                    Nv[u][m]=vectortemp[u];
                }
                N+=Nv.transposed();
            }


            //Now M
            Real productSD=0;

            Coord vectSD=tetInfo->m_SPKTensorGeneral*svk;
            productSD=dot(vectSD,svl);
            M[0][1]=M[0][2]=M[1][0]=M[1][2]=M[2][0]=M[2][1]=0;
            M[0][0]=M[1][1]=M[2][2]=(Real)productSD;

            edgeDfDx += (M+N)*tetInfo->m_restVolume;
        }// end of for j
    }//end of for i
    m_updateMatrix=false;
}


template <typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df, const DataVecDeriv& d_dx)
{
    VecDeriv& df = *d_df.beginEdit();
    const VecDeriv& dx = d_dx.getValue();
    Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    unsigned int l=0;
    unsigned int nbEdges=m_topology->getNbEdges();
    const vector< Edge> &edgeArray=m_topology->getEdges() ;

    helper::vector<EdgeInformation>& edgeInf = *(m_edgeInfo.beginEdit());

    EdgeInformation *einfo;


    /// if the  matrix needs to be updated
    if (m_updateMatrix) {
      this->updateTangentMatrix();
    }// end of if


    /// performs matrix vector computation
    unsigned int v0,v1;
    Deriv deltax;	Deriv dv0,dv1;

    for(l=0; l<nbEdges; l++ )
    {
        einfo=&edgeInf[l];
        v0=edgeArray[l][0];
        v1=edgeArray[l][1];

        deltax= dx[v0] - dx[v1];
        dv0 = einfo->DfDx * deltax;
        // do the transpose multiply:
        dv1[0] = (Real)(deltax[0]*einfo->DfDx[0][0] + deltax[1]*einfo->DfDx[1][0] + deltax[2]*einfo->DfDx[2][0]);
        dv1[1] = (Real)(deltax[0]*einfo->DfDx[0][1] + deltax[1]*einfo->DfDx[1][1] + deltax[2]*einfo->DfDx[2][1]);
        dv1[2] = (Real)(deltax[0]*einfo->DfDx[0][2] + deltax[1]*einfo->DfDx[1][2] + deltax[2]*einfo->DfDx[2][2]);
        // add forces
        df[v0] += dv1 * kFactor;
        df[v1] -= dv0 * kFactor;
    }
    m_edgeInfo.endEdit();
    m_tetrahedronInfo.endEdit();

    d_df.endEdit();
}

template<typename DataTypes>
SReal TetrahedronMooneyRivlinFEMForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams*, const DataVecCoord&) const
{
    msg_error() << "ERROR("<<this->getClassName()<<"): getPotentialEnergy( const MechanicalParams*, const DataVecCoord& ) not implemented.";
    return 0.0;
}

template <typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix *mat, SReal k, unsigned int &offset)
{
    /// if the  matrix needs to be updated
    if (m_updateMatrix)
    {
        this->updateTangentMatrix();
    }

    unsigned int nbEdges=m_topology->getNbEdges();
    const vector< Edge> &edgeArray=m_topology->getEdges() ;
    helper::vector<EdgeInformation>& edgeInf = *(m_edgeInfo.beginEdit());
    EdgeInformation *einfo;
    unsigned int i,j,N0, N1, l;
        Index noeud0, noeud1;

    for(l=0; l<nbEdges; l++ )
    {
        einfo=&edgeInf[l];
        noeud0=edgeArray[l][0];
        noeud1=edgeArray[l][1];
        N0 = offset+3*noeud0;
        N1 = offset+3*noeud1;

        for (i=0; i<3; i++)
        {
            for(j=0; j<3; j++)
            {
                mat->add(N0+i, N0+j,  einfo->DfDx[j][i]*k);
                mat->add(N0+i, N1+j, - einfo->DfDx[j][i]*k);
                mat->add(N1+i, N0+j, - einfo->DfDx[i][j]*k);
                mat->add(N1+i, N1+j, + einfo->DfDx[i][j]*k);
            }
        }
    }
    m_edgeInfo.endEdit();
}

template<typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::saveMesh( const char *filename )
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

template<typename DataTypes>
void TetrahedronMooneyRivlinFEMForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    //	unsigned int i;
    if (!vparams->displayFlags().getShowForceFields()) return;
    if (!this->mstate) return;

    vparams->drawTool()->saveLastState();

    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();

    if (vparams->displayFlags().getShowWireFrame())
          vparams->drawTool()->setPolygonMode(0,true);


    std::vector< Vector3 > points[4];
    for(Topology::TetrahedronID i = 0 ; i<m_topology->getNbTetrahedra();++i)
    {
        const Tetrahedron t=m_topology->getTetrahedron(i);

        Index a = t[0];
        Index b = t[1];
        Index c = t[2];
        Index d = t[3];
        Coord center = (x[a]+x[b]+x[c]+x[d])*0.125;
        Coord pa = (x[a]+center)*(Real)0.666667;
        Coord pb = (x[b]+center)*(Real)0.666667;
        Coord pc = (x[c]+center)*(Real)0.666667;
        Coord pd = (x[d]+center)*(Real)0.666667;

        points[0].push_back(pa);
        points[0].push_back(pb);
        points[0].push_back(pc);

        points[1].push_back(pb);
        points[1].push_back(pc);
        points[1].push_back(pd);

        points[2].push_back(pc);
        points[2].push_back(pd);
        points[2].push_back(pa);

        points[3].push_back(pd);
        points[3].push_back(pa);
        points[3].push_back(pb);
    }

    Vec<4,float> color1;
    Vec<4,float> color2;
    Vec<4,float> color3;
    Vec<4,float> color4;

    std::string material = d_materialName.getValue();
    if (material=="MooneyRivlin"){
        color1 = Vec<4,float>(0.0,1.0,0.0,1.0);
        color2 = Vec<4,float>(0.0,1.0,0.5,1.0);
        color3 = Vec<4,float>(0.0,1.0,1.0,1.0);
        color4 = Vec<4,float>(0.5,1.0,1.0,1.0);
    }
    else if (material=="MooneyRivlinIncompressible"){
        color1 = Vec<4,float>(0.0,1.0,0.0,1.0);
        color2 = Vec<4,float>(0.0,1.0,0.5,1.0);
        color3 = Vec<4,float>(0.0,1.0,1.0,1.0);
        color4 = Vec<4,float>(0.5,1.0,1.0,1.0);
    }
    else {
        color1 = Vec<4,float>(0.0,1.0,0.0,1.0);
        color2 = Vec<4,float>(0.5,1.0,0.0,1.0);
        color3 = Vec<4,float>(1.0,1.0,0.0,1.0);
        color4 = Vec<4,float>(1.0,1.0,0.5,1.0);
    }

    vparams->drawTool()->drawTriangles(points[0], color1);
    vparams->drawTool()->drawTriangles(points[1], color2);
    vparams->drawTool()->drawTriangles(points[2], color3);
    vparams->drawTool()->drawTriangles(points[3], color4);

    if (vparams->displayFlags().getShowWireFrame())
          vparams->drawTool()->setPolygonMode(0,false);

    vparams->drawTool()->restoreLastState();
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_INL
