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
  using namespace sofa::helper; // for M_PI: see rmath.h

template<typename DataTypes>
void IsochoricForceField<DataTypes>::SphericalPolarHandler::applyCreateFunction(unsigned int sphereIndex,
                                                                            SphericalPolarRestInformation &sinfo,
                                                                            const Sphere &,
                                                                            const sofa::helper::vector<unsigned int> &,
                                                                            const sofa::helper::vector<double> &)
{
  if (ff) {
      const vector< Sphere > &triArray=ff->m_topology->getTriangles() ;
      const std::vector< Edge> &edgeArray=ff->m_topology->getEdges() ;
      unsigned int j;
      // int k;
      typename DataTypes::Real volume;
      typename DataTypes::Coord point[3];
      const typename DataTypes::VecCoord restPosition = ff->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

      ///describe the indices of the 3 spherical coordinates at a point
      const Sphere &tri_array= triArray[sphereIndex];
      BaseMeshTopology::EdgesInTriangle te=ff->m_topology->getEdgesInTriangle(sphereIndex);

      // store the point position
      for(j=0;j<3;++j)
          point[j]=(restPosition)[tri_array[j]];

      sphInfo->m_R = rsqrt(SQ(point[0]), SQ(point[1]), SQ(point[2])); //(sqrt(x^2+y^2+z^2))
      sphinfo->m_Theta = std::atan(point[1]/point[0]);    // arctan(y/x)
      sphInfo->m_Phi = std::acos(point[2]/sphInfo->m_R); // arccos(z/r)
      // initialize associated tri components
      sphInfo->m_Ro = sphInfo->m_R;

      // assemble radial vector
      sinfo.m_RadialVector = {sinfo.m_R, sinfo.m_Theta, sinfo.m_Phi};
      /// store the rest volume
      sinfo.m_restVolume = std::fabs((2/3)*M_Pi*m_RadialVector.norm());

      for(j=0;j<6;++j) {
          Edge e=ff->m_topology->getLocalEdgesInTriangle(j);
          int k=e[0];
          //int l=e[1];
          if (edgeArray[te[j]][0]!=t[k]) {
              k=e[1];
              //l=e[0];
          }
      }
      // update Tangent Matrix
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
    , f_poisson(initData(&f_poisson,helper::vector<Real>(1,static_cast<Real>(0.45)),"poissonRatio","Poisson ratio in Hooke's law (vector)"))
    , f_young(initData(&f_young,helper::vector<Real>(1,static_cast<Real>(1000.0)),"youngModulus","Young modulus in Hooke's law (vector)"))
    , f_damping(initData(&f_damping,(Real)0.,"damping","Ratio damping/stiffness"))
    , m_sphericalPolarHandler(nullptr)
{
  m_sphericalPolarHandler = new SphericalPolarHandler(this, &m_sphericalPolarInfo);
  f_poisson.setRequired(true);
  f_young.setRequired(true);
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

    // this will be the spherical polar info for reference configuration
    helper::vector<typename IsochoricForceField<DataTypes>::SphericalPolarRestInformation>& triSphereInfVec = *(m_sphericalPolarInfo.beginEdit());

    triSphereInfVec->C1    = paramSet[0];
    triSphereInfVec->C2    = paramSet[1];
    /// prepare to store info in the triangle array
    triSphereInfVec.resize(m_topology->getNbTriangles());

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

    /// initialize the data structure associated with each triangle
    for (Topology::TriangleID triID=0; triID<m_topology->getNbTriangles(); ++triID)
    {
        m_sphericalPolarHandler->applyCreateFunction(triID, triSphereInfVec[triID],
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

template <class DataTypes>
void IsochoricForceField<DataTypes>::updateTangentMatrix()
{
    unsigned int i=0,j=0,k=0,l=0;
    unsigned int nbEdges=m_topology->getNbEdges();
    const vector< Edge> &edgeArray=m_topology->getEdges() ;

    helper::vector<EdgeInformation>& edgeInf = *(m_edgeInfo.beginEdit());
    helper::vector<SphericalPolarRestInformation>& sphereInfVec = *(m_sphericalPolarInfo.beginEdit());

    EdgeInformation *einfo;
    SphericalPolarRestInformation *sphInfo;
    unsigned int nbTriangles=m_topology->getNbTriangles();
    const std::vector< Sphere > &triSphTopo=m_topology->getTriangle() ;

    for(l=0; l<nbEdges; l++ )
      edgeInf[l].DfDx.clear();
    for(i=0; i<nbTriangles; i++ )
    {
        sphInfo=&sphereInfVec[i];
        Matrix3 &df=sphInfo->m_F;
//			Matrix3 Tdf=df.transposed();
        BaseMeshTopology::EdgesInTriangle tri_edges=m_topology->getEdgesInTriangle(i);

        /// describe the jth vertex index of triangle no i
        const Sphere &sph= triSphTopo[i];
        for(j=0;j<6;j++) {
            einfo= &edgeInf[tri_edges[j]];
            Edge e=m_topology->getLocalEdgesInTriangle(j);

            k=e[0];
            l=e[1];
            if (edgeArray[tri_edges[j]][0]!=sph[k]) {
                k=e[1];
                l=e[0];
            }
            Matrix3 &edgeDfDx = einfo->DfDx;

/* What are these for? Review tonight
            Coord svl=sphInfo->m_shapeVector[l];
            Coord svk=sphInfo->m_shapeVector[k];

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

                m_myMaterial->applyElasticityTensor(sphInfo,globalParameters,inputTensor[m],outputTensor);
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

            Coord vectSD=sphInfo->m_SPKTensorGeneral*svk;
            productSD=dot(vectSD,svl);
            M[0][1]=M[0][2]=M[1][0]=M[1][2]=M[2][0]=M[2][1]=0;
            M[0][0]=M[1][1]=M[2][2]=(Real)productSD;

            edgeDfDx += (M+N)*sphInfo->m_restVolume;
*/
        }// end of for j
    }//end of for i
    m_updateMatrix=false;
}


template<typename DataTypes>
void IsochoricForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                             DataVecDeriv& d_f, const DataVecCoord& d_x,
                                             const DataVecDeriv& /*d_v*/)
{
    sofa::helper::AdvancedTimer::stepBegin("addForceSphericalPolarFEM");

    VecDeriv& f = *d_f.beginEdit();
    const VecCoord& x = d_x.getValue();

    unsigned int i=0,j=0,k=0,l=0;
    unsigned int nbTriangles=m_topology->getNbTriangles();

    // contains information about all sphere fems in an IAB as a vector
    helper::vector<SphericalPolarRestInformation>& sphereInfVec = *(m_sphericalPolarInfo.beginEdit());
    // the spherical info at the current configuration
    SphericalPolarRestInformation *sphInfo;

    assert(this->mstate, "No rest state specified");

    // opportunity for improvement using my formulation here
    for(i=0; i<nbTriangles; i++ )
    {
      const Sphere &triSphTopo= m_topology->getTriangle(i);
      sphInfo=&sphereInfVec[i];
      /* see http://mathworld.wolfram.com/SphericalCoordinates.html
      My convention is (x, y, z)->(triSphTopo[0], triSphTopo[1], triSphTopo[2])
      */
      sphInfo->m_r = rsqrt(SQ(x[triSphTopo[0]]), SQ(x[triSphTopo[1]), SQ(x[triSphTopo[2]]));
      sphinfo->m_theta = std::atan(SQ(x[triSphTopo[1]]/SQ(x[triSphTopo[0]]); // atan(y/x)
      sphInfo->m_phi = std::acos(x[triSphTopo[2]]/sphInfo->m_r]); // acos(z/r)
      // initialize associated tri components
      sphInfo->m_ro = sphInfo->m_r;
      // compute F, B, and C (defGrad, left and right Cauchy-Green Tensors)
      this->updateStrainInfo(std::move(i), std::move(sphInfo),std::move(sphereInfVec));
      // now fix generalized coordinates for shear deformation
    }

    /// indicates that the next call to addDForce will need to update the stiffness matrix
    m_updateMatrix=true;
    m_sphericalPolarInfo.endEdit();
    d_f.endEdit();
}


template<typename DataTypes>
void IsochoricForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams,
                                              DataVecDeriv& d_f , const DataVecDeriv& d_x)
{
    // Compute the force derivative d_df from the current, which will be multiplied with the field d_dx
    VecDeriv& f = *d_f.beginEdit();
    const VecCoord& x = d_x.getValue();
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

    const bool printLog = this->f_printLog.getValue();
    if (printLog && !m_meshSaved)
    {
        saveMesh( "/opt/sofa-result.stl" );
        printf( "Mesh saved.\n" );
        m_meshSaved = true;
    }
    unsigned int i=0,j=0,k=0,l=0;
    unsigned int nbTriangles=m_topology->getNbTriangles();

    helper::vector<SphericalPolarRestInformation>& sphereInfVec = *(m_sphericalPolarInfo.beginEdit());

    SphericalPolarRestInformation *sphInfo;

    assert(this->mstate);

    // opportunity for improvement using my formulation here
    for(i=0; i<nbTriangles; i++ )
    {
      const Sphere &triSphTopo= m_topology->getTriangle(i);
      sphInfo=&sphereInfVec[i];
      /* see http://mathworld.wolfram.com/SphericalCoordinates.html
      My convention is (x, y, z)->(triSphTopo[0], triSphTopo[1], triSphTopo[2])
      */
      sphInfo->m_r = rsqrt(SQ(x[triSphTopo[0]]), SQ(x[triSphTopo[1]), SQ(x[triSphTopo[2]]));
      sphinfo->m_theta = std::atan(SQ(x[triSphTopo[1]]/SQ(x[triSphTopo[0]]); // atan(y/x)
      sphInfo->m_phi = std::acos(x[triSphTopo[2]]/sphInfo->m_r]); // acos(z/r)
      // initialize associated tri components
      sphInfo->m_ro = sphInfo->m_r;
      // compute F, B, and C (defGrad, left and right Cauchy-Green Tensors)
      this->updateStrainInfo(std::move(sphInfo));
      // now fix generalized coordinates for shear deformation
    }

    /// indicates that the next call to addDForce will need to update the stiffness matrix
    m_updateMatrix=true;
    m_sphericalPolarInfo.endEdit();
    d_f.endEdit();
}

void updateStrainInfo(SphericalPolarRestInformation&& sphInfo)
{
  // compute the deformation gradient
  // initialize bottom left block of def grad to zeros
  for (int k = 1; k < 3; ++k) // rows 1 and 2
    for(int l=0; l <2; ++l)  // cols 0 through 1
      sphInfo->m_F[k][l] =  0;
  sphInfo->m_F[0][0] = SQ(sphInfo->m_R)/SQ(sphInfo->m_r);
  sphInfo->m_F[0][1] = -sphInfo->m_phi/sphInfo->m_R;
  sphInfo->m_F[0][2] = -sphInfo->m_theta/sphInfo->m_R;
  sphInfo->m_F[1][1] = (sphInfo->m_r/sphInfo->m_R)+(1/sphInfo->m_R);
  sphInfo->m_F[1][2] = -(sphInfo->m_theta/sphInfo->m_R)*\
                        (std::cos(sphInfo->m_phi)/std::sin(sphInfo->m_phi));
  sphInfo->m_F[2][2] = (sphInfo->m_r/sphInfo->m_R)+\
                        (sphInfo->m_phi/sphInfo->m_R)*\
                        (std::cos(sphInfo->m_phi)/std::sin(sphInfo->m_phi))+\
                        ((1/sphInfo->R)*sphInfo->m_Phi);

  /* left and right cauchy-green tensors
  only diagonal elements are non-zero
  */
  int idx = 0;
  while(idx < 3)
  {
    sphInfo->m_C[idx][idx] = SQ(sphInfo->m_F[idx][idx])
    sphInfo->m_B[idx][idx] = SQ(sphInfo->m_F[idx][idx])
    ++idx;
  }
  // compute fiber direction
  this->m_fiber_Direction = {sphInfo->m_F[0][0]*rcos(sphinfo->gamma), -sphInfo->m_F[1][1]*sphinfo->gamma, 0}
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
  // nothing to do here
};

template<typename DataTypes>
void IsochoricForceField<DataTypes>::computePositionalVector()
{
    // Compute the positional vector for every point on the spherical body
    // using Matrix2 = Mat<2,1, Real>;
    // this->m_radialVector.resize(2);
    this->m_radialVector(this->m_r/this->m_R, this_>m_beta-this->m_alpha);
}

template<typename DataTypes>
void IsochoricForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix *  mat ,
                                                 SReal  k , unsigned int &  offset )
{
    // Compute the force derivative d_df from the current and store the resulting matrix

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
