/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture                          *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                           Plugin SoftRobots    v1.0                         *
*				                                              *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/
#include "IABPlugin/behavior/IABBaseConstraint.h"

namespace sofa
{

namespace core
{

namespace behavior
{

using helper::vector;

IABBaseConstraint::IABBaseConstraint()
    : m_hasDeltaMax(false)
    , m_hasDeltaMin(false)
    , m_hasDeltaEqual(false)
    , m_hasLambdaMax(false)
    , m_hasLambdaMin(false)
    , m_hasLambdaEqual(false)
    , m_hasEpsilon(false)
{
}

bool IABBaseConstraint::hasDeltaMax()
{
    return m_hasDeltaMax;
}

bool IABBaseConstraint::hasDeltaMin()
{
    return m_hasDeltaMin;
}

bool IABBaseConstraint::hasDeltaEqual()
{
    return m_hasDeltaEqual;
}



bool IABBaseConstraint::hasLambdaMax()
{
    return m_hasLambdaMax;
}

bool IABBaseConstraint::hasLambdaMin()
{
    return m_hasLambdaMin;
}

bool IABBaseConstraint::hasLambdaEqual()
{
    return m_hasLambdaEqual;
}


bool IABBaseConstraint::hasEpsilon()
{
    return m_hasEpsilon;
}


SReal IABBaseConstraint::getDeltaMax()
{
    return m_deltaMax;
}

SReal IABBaseConstraint::getDeltaMin()
{
    return m_deltaMin;
}

SReal IABBaseConstraint::getDeltaEqual()
{
    return m_deltaEqual;
}



SReal IABBaseConstraint::getLambdaMax()
{
    return m_lambdaMax;
}

SReal IABBaseConstraint::getLambdaMin()
{
    return m_lambdaMin;
}

SReal IABBaseConstraint::getLambdaEqual()
{
    return m_lambdaEqual;
}


SReal IABBaseConstraint::getEpsilon()
{
    return m_epsilon;
}


unsigned int IABBaseConstraint::getNbLines()
{
    return m_nbLines;
}

void IABBaseConstraint::storeResults(vector<double>& lambda, vector<double> &delta)
{
    SOFA_UNUSED(lambda);
    SOFA_UNUSED(delta);
}

void IABBaseConstraint::storeResults(vector<double> &delta)
{
    SOFA_UNUSED(delta);
}

} // namespace behavior

} // namespace core

} // namespace sofa
