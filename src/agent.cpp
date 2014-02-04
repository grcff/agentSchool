#include "agent.h"
#include "tools.cpp"
#include <stdio.h>

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
{
    nXVec_.resize(0.);
    nYVec_.resize(0.);
    nYawVec_.resize(0.);
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

void Agent::updatePos(Scalar t)
{
    // Reset gradient and hessian_
    hessian_ = 0.;
    gradient_ = 0.;

    // Compute weights
    xGetBackPredatorWeight();
    xGetPredatorDistanceWeight();


    // Building hessian and gradient
    // If none of the objectives have a significant influence, the agent just wander around

    if(backPredatorWeight_ < EPSILON)
    {
        //std::cout << "DUDE"<<std::endl;
        hessian_ = xGetWanderAngleHessian(t);
        gradient_ = xGetWanderAngleGradient(t);
    }
    else
    {
        hessian_ += backPredatorWeight_*xGetBackPredatorHessian(t);
        gradient_ += backPredatorWeight_*xGetBackPredatorGradient(t);
    }

    // Updtating yaw coordinate
    yaw_ += Solver::solve(hessian_, 2*gradient_, wMax_, -wMax_)*t;

    // Reset gradient and hessian_
    hessian_ = 0.;
    gradient_ = 0.;

    if(predatorDistanceWeight_ < EPSILON)
    {
        hessian_ = xGetWanderHessian(t);
        gradient_ = xGetWanderGradient(t);
    }
    else
    {
        hessian_ += predatorDistanceWeight_*xGetPredatorDistanceHessian(t);
        gradient_ += predatorDistanceWeight_*xGetPredatorDistanceHessian(t);
    }

    // Updating x and y coordinate
    Scalar sol = Solver::solve(hessian_, 2*gradient_, std::min(xGetVMaxBound(t), vMax_), 0.);
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

void Agent::addNeigbor(const Agent& agent)
{
    int oldSize = nXVec_.rows();
    nXVec_.conservativeResize(oldSize + 1);
    nYVec_.conservativeResize(oldSize + 1);
    nYawVec_.conservativeResize(oldSize + 1);
    nXVec_(oldSize) = agent.getX();
    nYVec_(oldSize) = agent.getY();
    nYawVec_(oldSize) = agent.getYaw();
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

Scalar Agent::xGetMeanDirectionHessian(Scalar t)
{
    return t*t;
}

Scalar Agent::xGetMeanDirectionGradient(Scalar t)
{
    int m = nYawVec_.rows();
    Scalar meanYaw = yaw_;

    for(int i=0; i<m; ++i)
    {
        meanYaw += nYawVec_(i);
    }

    return t*(yaw_ - (1/(m + 1))*(meanYaw));
}

void Agent::xGetMeanDirectionWeight()
{
    meanDirectionWeight_ = 0;
}


Scalar Agent::xGetBackPredatorHessian(Scalar t)
{
    return t*t;
}

Scalar Agent::xGetBackPredatorGradient(Scalar t)
{
    return t*(xWrapAngle(yaw_) - xWrapAngle(std::atan2(yP_- y_, xP_- x_) + M_PI));
}

void Agent::xGetBackPredatorWeight()
{
    if(std::pow((x_ - xP_), 2) + std::pow((y_ - yP_), 2) < std::pow(sightHorizon_, 2))
        //TODO: sightHorizon_ pourrait être déterminé par l'algo gen
    {
        backPredatorWeight_ = 1;
    }
    else
    {
        backPredatorWeight_ = 0.;
    }
}

Scalar Agent::xGetBarycenterHessian(Scalar t)
{
    return t*t;
}

Scalar Agent::xGetBarycenterGradient(Scalar t)
{

    int m = nXVec_.rows();
    Scalar meanX = x_;
    Scalar meanY = y_;

    for(int i=0; i<m; ++i)
    {
        meanX += nXVec_(i);
        meanY += nYVec_(i);
    }

    return t*(std::cos(yaw_)*(x_ - (1/(m + 1))*meanX) +
              std::sin(yaw_)*(y_ - (1/(m + 1))*meanY));
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
        predatorDistanceWeight_ = 1;
    }
    else
    {
        predatorDistanceWeight_ = 0.;
    }
}

// TODO: delete?
Scalar Agent::xGetLowSpeedHessian(Scalar t)
{
    return 1.;
}

Scalar Agent::xGetLowSpeedGradient(Scalar t)
{
    return 0.;
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
    Scalar wrappedYaw = xWrapAngle(yaw_);
    Scalar d = size_/10.; // arbitrary...

    if(std::abs(x_ - box_.xMax_) < d)
    {
        return -t*(wrappedYaw);;
    }
    else if(std::abs(x_ - box_.xMin_) < d)
    {
        return -t*(wrappedYaw - M_PI);
    }
    else if(std::abs(y_ - box_.yMax_) < d)
    {
        return -t*(wrappedYaw - M_PI/2.);
    }
    else if(std::abs(y_ - box_.yMin_) < d)
    {
        return -t*(wrappedYaw - 3.*M_PI/2.);
    }

    return 0.;
}

Scalar Agent::xGetWanderHessian(Scalar t)
{
    return 1.;
}

Scalar Agent::xGetWanderGradient(Scalar t)
{
    return -vMax_/2.;
}

Scalar Agent::xGetVMaxBound(Scalar t)
{
    if(std::cos(yaw_) > EPSILON)
    {
        if(std::sin(yaw_) > EPSILON)
        {
            return std::min((box_.xMax_ - x_)/(std::cos(yaw_)*t),
                            (box_.yMax_ - y_)/(std::sin(yaw_)*t));
        }
        else if(std::sin(yaw_) < EPSILON)
        {
            return std::min((box_.xMax_ - x_)/(std::cos(yaw_)*t),
                            (box_.yMin_ - y_)/(std::sin(yaw_)*t));

        }
        else
        {
            return (box_.xMax_ - x_)/(std::cos(yaw_)*t);
        }
    }
    else if(std::cos(yaw_) < EPSILON)
    {
        if(std::sin(yaw_) > EPSILON)
        {
            return std::min((box_.xMin_ - x_)/(std::cos(yaw_)*t),
                            (box_.yMax_ - y_)/(std::sin(yaw_)*t));
        }
        else if(std::sin(yaw_) < EPSILON)
        {
            return std::min((box_.xMin_ - x_)/(std::cos(yaw_)*t),
                            (box_.yMin_ - y_)/(std::sin(yaw_)*t));
        }
        else
        {
            return (box_.xMin_ - x_)/(std::cos(yaw_)*t);
        }
    }
    else
    {
        if(std::sin(yaw_) > EPSILON)
        {
            return (box_.yMax_ - y_)/(std::sin(yaw_)*t);
        }
        else
        {
            return (box_.yMin_ - y_)/(std::sin(yaw_)*t);
        }

    }
}

Scalar Agent::xWrapAngle(Scalar angle)
{
    return angle - 2*M_PI*std::floor(angle/(2*M_PI));
}

Scalar Agent::xGetClosestAngle(Scalar angle, Scalar reference)
{
    if(std::abs(angle - reference) < M_PI)
    {
        return angle;
    }
    else
    {
        if(angle >= reference)
        {
            return angle - 2*M_PI;
        }
        else
        {
            return angle + 2*M_PI;
        }
    }
}

