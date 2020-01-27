#define SOFA_CONTROLLER_ANIMATIONEDITOR_CPP

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Quat.h>

#include "IABPlugin/controller/AnimationEditor.inl"

namespace sofa
{

namespace component
{

namespace controller
{

using sofa::defaulttype::Quat;
using sofa::defaulttype::Vec3Types ;
using sofa::defaulttype::Rigid3Types ;


namespace _animationeditor_ {

template<>
void AnimationEditor<Rigid3Types>::updateAnimationWithInterpolation(const int startKey,
                                                                     const int endKey)
{
    if(m_state == nullptr)
        return;

    vector<Coord> previousPositions = m_animation[startKey];
    vector<Coord> currentPositions  = m_animation[endKey];

    if (currentPositions.size() != previousPositions.size())
    {
        msg_warning() <<"This component does not handle mechanical state size changes";
        return;
    }

    int nbPositions = m_state->read(core::ConstVecCoordId::position())->getValue().size();
    int nbStep = endKey - startKey;

    for (int i=0; i<nbStep+1; i++)
    {
        vector<Coord> newPositions;
        for (int k=0; k<nbPositions; k++)
        {
            Rigid3Types::CPos direction = currentPositions[k].getCenter() - previousPositions[k].getCenter();
            double distance = direction.norm();
            double step = distance/nbStep*i;
            direction.normalize();
            Rigid3Types::CPos center =  previousPositions[k].getCenter() + direction*step;

            double h =  (double)i/(double)nbStep;
            Quat orientation;
            for(int l=0; l<4; l++)
                orientation[l] = previousPositions[k].getOrientation()[l]*(1.-h) + currentPositions[k].getOrientation()[l]*h;
            orientation.normalize();

            Coord newPosition;
            Rigid3Types::setCPos(newPosition, center);
            Rigid3Types::setCRot(newPosition, orientation);
            newPositions.push_back(newPosition);
        }
        m_animation[startKey+i] = newPositions;
    }
}


template<>
void AnimationEditor<Rigid3Types>::drawTrajectory(const VisualParams* vparams)
{
    vector<Rigid3Types::CPos> points;
    vector<unsigned int> IDSorted = m_keyFramesID;
    unsigned int nbKey = m_keyFramesID.size();
    std::sort(IDSorted.begin(), IDSorted.begin() + nbKey);
    for(unsigned int i=0; i<nbKey; i++)
    {
        for(unsigned int k=0; k<m_animation[i].size(); k++)
        {
            points.push_back(m_animation[IDSorted[i]][k].getCenter());
            vparams->drawTool()->drawFrame(m_animation[m_keyFramesID[i]][k].getCenter(), m_animation[m_keyFramesID[i]][k].getOrientation(), Rigid3Types::CPos(d_drawSize.getValue(),d_drawSize.getValue(),d_drawSize.getValue()));
        }
    }

    vector<Rigid3Types::CPos> lines;
    for(unsigned int i=0; i<points.size()-1; i++)
    {
        lines.push_back(points[i]);
        lines.push_back(points[i+1]);
    }

    vparams->drawTool()->drawLines(lines,d_drawSize.getValue()*2.,Vec4f(0.5,0.5,0.5,1.));
}


////////////////////////////////////////////    FACTORY    ////////////////////////////////////////////
using sofa::core::RegisterObject ;
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int AnimationEditorClass = RegisterObject("Build an animation from key points motion: \n"
                                   "ctrl+a: add keyframe \n"
                                   "ctrl+d: delete keyframe \n"
                                   "ctrl+c: copy keyframe \n"
                                   "ctrl+v: paste keyframe \n"
                                   "ctrl+x: cut keyframe \n"
                                   "ctrl+w: write animation \n"
                                   "ctrl+m: play/pause animation \n"
                                   "ctrl+(left/right)arrow: move the cursor along the timeline \n"
                                   "ctrl+(pgDn/pgUp): move the cursor to the next/previous keyframe")
///////////////////////////////////////////////////////////////////////////////////////////////////////

        .add< AnimationEditor<Vec3Types> >(true)
        .add< AnimationEditor<Rigid3Types> >()

        ;

template class SOFA_IABPlugin_API AnimationEditor<Vec3Types>;
template class SOFA_IABPlugin_API AnimationEditor<Rigid3Types>;



}//namespace _animationeditor_
}//namespace controller
}//namespace component
}//namespace sofa
