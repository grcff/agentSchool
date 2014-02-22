#include "agent.h"
#include "tools.cpp"
#include <stdio.h>
//DEBUG
#include <typeinfo>

Agent::Agent()
    :x_(0.)
    ,y_(0.)
    ,yaw_(0.)
    ,xP_(0.)
    ,yP_(0.)
    ,size_(0.)
    ,vMax_(0.)
    ,wMax_(0.)
    ,sightHorizon_(0.)
    ,sightMaxAngle_(0.)
    ,hessian_(0.)
    ,gradient_(0.)
    ,box_()
    ,meanDirectionWeight_(0.)
    ,backPredatorWeight_(0.)
    ,wanderAngleWeight_(0.)
    ,barycenterWeight_(0.)
    ,predatorDistanceWeight_(0.)
    ,wanderWeight_(0.)
{
    nXVec_.resize(0.);
    nYVec_.resize(0.);
    nYawVec_.resize(0.);
    antiStackWeightVec_.resize(0.);
}

Agent::Agent(Scalar size,
             Scalar vMax,
             Scalar wMax,
             Scalar sightHorizon,
             Scalar sightMaxAngle,
             Box box)
    :x_(0.)
    ,y_(0.)
    ,yaw_(0.)
    ,xP_(0.)
    ,yP_(0.) //change to box.yMax_ + sightHorizon
    ,size_(size)
    ,vMax_(vMax)
    ,wMax_(wMax)
    ,sightHorizon_(sightHorizon)
    ,sightMaxAngle_(sightMaxAngle)
    ,hessian_(0.)
    ,gradient_(0.)
    ,box_(box)
    ,meanDirectionWeight_(0.)
    ,backPredatorWeight_(0.)
    ,wanderAngleWeight_(0.)
    ,barycenterWeight_(0.)
    ,predatorDistanceWeight_(0.)
    ,wanderWeight_(0.)
{
    assert(vMax > -EPSILON);
    assert(wMax > -EPSILON);
    nXVec_.resize(0.);
    nYVec_.resize(0.);
    nYawVec_.resize(0.);
}


Agent::~Agent()
{
}

/*********************************************/
/************** UPDATE POSITION **************/
/*********************************************/

void Agent::updatePos(Scalar t)
{
    // Wrap yaw_ between 0 and 2Pi
    yaw_ = Tools::wrapAngle(yaw_);

    // Compute weights
    xGetBackPredatorWeight();
    xGetMeanDirectionWeight();
    xGetWanderAngleWeight();

    xGetBarycenterWeight();
    xGetPredatorDistanceWeight();
    xGetAntiStackingWeight();
    xGetWanderWeight();

    /************ ANGLE ***********/
    // Building hessian and gradient
    hessian_ = backPredatorWeight_*xGetBackPredatorHessian(t)
            + meanDirectionWeight_*xGetMeanDirectionHessian(t)
            + wanderAngleWeight_*xGetWanderAngleHessian(t);
    gradient_ = backPredatorWeight_*xGetBackPredatorGradient(t)
            + meanDirectionWeight_*xGetMeanDirectionGradient(t)
            + wanderAngleWeight_*xGetWanderAngleGradient(t) ;

    // Updtating yaw coordinate
    yaw_ += Tools::solve(hessian_, 2*gradient_, wMax_, -wMax_)*t;

    /************ TRANSLATION ***********/
    hessian_ = predatorDistanceWeight_*xGetPredatorDistanceHessian(t)
            + barycenterWeight_*xGetBarycenterHessian(t)
            + wanderWeight_*xGetWanderHessian(t);
    gradient_ = predatorDistanceWeight_*xGetPredatorDistanceGradient(t)
            + barycenterWeight_*xGetBarycenterGradient(t)
            + wanderWeight_*xGetWanderGradient(t);

    // Updating x and y coordinate
    Scalar sol = Tools::solve(hessian_, 2*gradient_, std::min(xGetVMaxBound(t), vMax_), 0.);

    x_ += sol*std::cos(yaw_)*t;
    y_ += sol*std::sin(yaw_)*t;
}

void Agent::setX(Scalar x)
{
    assert(x == x);
    x_ = x;
}

void Agent::setY(Scalar y)
{
    assert(y == y);
    y_ = y;
}

void Agent::setYaw(Scalar yaw)
{
    assert(yaw == yaw);
    yaw_ = yaw;
}

void Agent::setXP(Scalar xP)
{
    assert(xP == xP);
    xP_ = xP;
}


void Agent::setYP(Scalar yP)
{
    assert(yP == yP);
    yP_ = yP;
}

void Agent::cleanNeigborVectors()
{
    nXVec_.resize(0);
    nYVec_.resize(0);
    nYawVec_.resize(0);
}

void Agent::addNeigbor(const Agent& agent)
{
    size_t oldSize = nXVec_.size();
    nXVec_.resize(oldSize + 1);
    nYVec_.resize(oldSize + 1);
    nYawVec_.resize(oldSize + 1);
    nXVec_[oldSize] = agent.getX();
    nYVec_[oldSize] = agent.getY();
    nYawVec_[oldSize] = agent.getYaw();
}

/*********************************************/
/******************** ANGLE ******************/
/*********************************************/

Scalar Agent::xGetMeanDirectionHessian(Scalar t)
{
    return t*t;
}

Scalar Agent::xGetMeanDirectionGradient(Scalar t)
{
    size_t m = nYawVec_.size();

    if(m>0)
    {
        Scalar meanYaw = yaw_;

        for(size_t i=0; i<m; ++i)
        {
            meanYaw += Tools::getClosestAngle(nYawVec_[i], yaw_);
        }

        return t*(yaw_ - (1/static_cast<Scalar>(m+1))*meanYaw);
    }

    return 0.;
}

void Agent::xGetMeanDirectionWeight()
{
    if(nYawVec_.size() == 0)
    {
        meanDirectionWeight_ = 0.;
    }
    else
    {
        meanDirectionWeight_ = 10.;
    }
}


Scalar Agent::xGetBackPredatorHessian(Scalar t)
{
    return t*t;
}

Scalar Agent::xGetBackPredatorGradient(Scalar t)
{
    return t*(yaw_ - Tools::getClosestAngle(Tools::wrapAngle(std::atan2(yP_- y_, xP_- x_) + M_PI), yaw_));
}

void Agent::xGetBackPredatorWeight()
{
    if(std::pow((x_ - xP_), 2) + std::pow((y_ - yP_), 2) < std::pow(2*sightHorizon_, 2))
        //TODO: sightHorizon_ pourrait être déterminé par l'algo gen
    {
        backPredatorWeight_ = 100.;
    }
    else
    {
        backPredatorWeight_ = 0.;
    }
}

Scalar Agent::xGetWanderAngleHessian(Scalar t)
{
    Scalar d = size_/10.; // arbitrary...
    if(std::abs(x_ - box_.xMax_) < d ||
            std::abs(x_ - box_.xMin_) < d ||
            std::abs(y_ - box_.yMax_) < d ||
            std::abs(y_ - box_.yMin_) < d)
    {
        return -t*t;
    }

    return 1.;
}

Scalar Agent::xGetWanderAngleGradient(Scalar t)
{
    Scalar d = size_/10.; // arbitrary...

    if(std::abs(x_ - box_.xMax_) < d)
    {
        return -t*(yaw_ - Tools::getClosestAngle(0., yaw_));
    }
    else if(std::abs(x_ - box_.xMin_) < d)
    {
        return -t*(yaw_ - Tools::getClosestAngle(M_PI, yaw_));
    }
    else if(std::abs(y_ - box_.yMax_) < d)
    {
        return -t*(yaw_ - Tools::getClosestAngle(M_PI/2., yaw_));
    }
    else if(std::abs(y_ - box_.yMin_) < d)
    {
        return -t*(yaw_ - Tools::getClosestAngle(3.*M_PI/2., yaw_));
    }

    return 0.;
}


void Agent::xGetWanderAngleWeight()
{
    wanderAngleWeight_ = 1;
}

/*********************************************/
/**************** TRANSLATION ****************/
/*********************************************/


Scalar Agent::xGetBarycenterHessian(Scalar t)
{
    return t*t;
}

Scalar Agent::xGetBarycenterGradient(Scalar t)
{

    size_t m = nXVec_.size();
    Scalar meanX = x_;
    Scalar meanY = y_;

    for(size_t i=0; i<m; ++i)
    {
        meanX += nXVec_[i];
        meanY += nYVec_[i];
    }

    return t*(std::cos(yaw_)*(x_ - (1./static_cast<Scalar>(m + 1))*meanX) +
              std::sin(yaw_)*(y_ - (1./static_cast<Scalar>(m + 1))*meanY));
}

void Agent::xGetBarycenterWeight()
{
    if(nXVec_.size() == 0)
    {
        barycenterWeight_ = 0.;
    }
    else
    {
        barycenterWeight_ = 10;
    }
}

Scalar Agent::xGetPredatorDistanceHessian(Scalar t)
{
    return -t*t;
}

Scalar Agent::xGetPredatorDistanceGradient(Scalar t)
{
    return t*(std::cos(yaw_)*(xP_ - x_) + std::sin(yaw_)*(yP_ - y_));
}

void Agent::xGetPredatorDistanceWeight()
{
    if(std::pow((x_ - xP_), 2) + std::pow((y_ - yP_), 2) < std::pow(sightHorizon_, 2))
        //TODO: sightHorizon_ pourrait être déterminé par l'algo gen
    {
        predatorDistanceWeight_ = 100.;
    }
    else
    {
        predatorDistanceWeight_ = 0.;
    }
}

//TODO: complete
Scalar Agent::xGetAntiStackingHessian(Scalar t)
{
    return 0.;
}
//TODO: complete
Scalar Agent::xGetAntiStackingGradient(Scalar t)
{
    return 0.;
}

//TODO: complete
void Agent::xGetAntiStackingWeight()
{
    size_t m = nYawVec_.size();
    antiStackWeightVec_.resize(m);
    for(size_t i=0; i<m; i++)
    {
        //antiStackWeightVec_[i] = Tools::sigmoid(1000, 25, )
    }
}


Scalar Agent::xGetWanderHessian(Scalar t)
{
    return 1.;
}

Scalar Agent::xGetWanderGradient(Scalar t)
{
    return -vMax_/1.5; //arbitrary...
}

void Agent::xGetWanderWeight()
{
    wanderWeight_ = 1;
}

Scalar Agent::xGetVMaxBound(Scalar t)
{
    Scalar d = 1;

    if(std::cos(yaw_) > EPSILON)
    {
        if(std::sin(yaw_) > EPSILON)
        {
            return std::min((box_.xMax_ - d - x_)/(std::cos(yaw_)*t),
                            (box_.yMax_ - d - y_)/(std::sin(yaw_)*t));

        }
        else if(std::sin(yaw_) < -EPSILON)
        {
            return std::min((box_.xMax_ - d - x_)/(std::cos(yaw_)*t),
                            (box_.yMin_ + d - y_)/(std::sin(yaw_)*t));
        }
        else
        {
            return (box_.xMax_ - d - x_)/(std::cos(yaw_)*t);
        }
    }
    else if(std::cos(yaw_) < -EPSILON)
    {
        if(std::sin(yaw_) > EPSILON)
        {
            return std::min((box_.xMin_ + d - x_)/(std::cos(yaw_)*t),
                            (box_.yMax_ - d - y_)/(std::sin(yaw_)*t));
        }
        else if(std::sin(yaw_) < -EPSILON)
        {
            return std::min((box_.xMin_ + d - x_)/(std::cos(yaw_)*t),
                            (box_.yMin_ + d - y_)/(std::sin(yaw_)*t));
        }
        else
        {
            return (box_.xMin_ + d - x_)/(std::cos(yaw_)*t);
        }
    }
    else
    {
        if(std::sin(yaw_) > EPSILON)
        {
            return (box_.yMax_ -d - y_)/(std::sin(yaw_)*t);
        }
        else
        {
            return (box_.yMin_ + d - y_)/(std::sin(yaw_)*t);
        }
    }
}

