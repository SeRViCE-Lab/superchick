/******************************************************************************
******************************************************************************/

#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREMODEL_H
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREMODEL_H

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/Vec3Types.h>

#include "IABPlugin/behavior/IABConstraint.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::topology::BaseMeshTopology ;
using sofa::core::behavior::IABConstraint ;
using sofa::core::visual::VisualParams ;
using sofa::core::objectmodel::Data ;
using sofa::defaulttype::Vec3dTypes ;
using sofa::defaulttype::Vec3fTypes ;
using sofa::defaulttype::BaseVector ;
using sofa::core::ConstraintParams ;
using sofa::helper::ReadAccessor ;
using sofa::core::VecCoordId ;


/**
 * This class contains common implementation of surface pressure constraints
*/
template< class DataTypes >
class SOFA_IABPlugin_API SurfacePressureModel : virtual public IABConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureModel,DataTypes),
               SOFA_TEMPLATE(IABConstraint,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;
    typedef helper::vector<unsigned int> SetIndexArray;

    typedef core::topology::BaseMeshTopology::Triangle      Triangle;
    typedef core::topology::BaseMeshTopology::Quad          Quad;
    typedef core::topology::BaseMeshTopology::Edge          Edge;

public:
    SurfacePressureModel(MechanicalState* object = nullptr);
    ~SurfacePressureModel() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void bwdInit() override;
    void reset() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Actuator //////////////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                                       DataMatrixDeriv &cMatrix,
                                       unsigned int &cIndex,
                                       const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    void storeLambda(const ConstraintParams* cParams,
                             core::MultiVecDerivId res,
                             const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////

protected:
    Data<helper::vector<Triangle> >     d_triangles;
    Data<helper::vector<Quad> >         d_quads;
    helper::vector<Edge>                m_edges;

    Data<Real>                          d_initialCavityVolume;
    Data<Real>                          d_cavityVolume;
    Data<bool>                          d_flipNormal;

    Data<double>                        d_pressure;
    Data<Real>                          d_maxPressure;
    Data<Real>                          d_minPressure;
    Data<double>                        d_volumeGrowth;
    Data<Real>                          d_maxVolumeGrowth;
    Data<Real>                          d_minVolumeGrowth;
    Data<Real>                          d_maxVolumeGrowthVariation;

    Data<bool>                          d_drawPressure;
    Data<Real>                          d_drawScale;
    // To remove in SoftRobots v20.0
    Data<bool>                          d_visualizationDepracated;
    Data<Real>                          d_showVisuScaleDepracated;
    //


protected:

    SReal getCavityVolume(const VecCoord &positions);
    void drawQuads(const VisualParams* vparams, float red, float green, float blue);
    void drawTriangles(const VisualParams* vparams, float red, float green, float blue);
    void drawLines(const VisualParams* vparams, float red, float green, float blue);
    std::string getValueString(Real pressure);

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using IABConstraint<DataTypes>::m_state ;
    using IABConstraint<DataTypes>::getContext ;
    using IABConstraint<DataTypes>::m_nbLines ;
    using IABConstraint<DataTypes>::m_constraintId ;
    using IABConstraint<DataTypes>::m_componentstate ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void setUpData();
    void internalInit();

    void drawValue(const core::visual::VisualParams* vparams);
    void computeEdges();
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class SurfacePressureModel<defaulttype::Vec3Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREMODEL_H
