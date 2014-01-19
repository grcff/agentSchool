#include "agent.h"
#include "tools.cpp"

Agent::Agent()
    :x_(0.)
    ,y_(0.)
    ,yaw_(0.)
    ,xP_(0.)
    ,yP_(0.)
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

Agent::Agent(Scalar x,
             Scalar y,
             Scalar yaw,
             Scalar vMax,
             Scalar wMax,
             Scalar sightHorizon,
             Scalar sightMaxAngle,
             Box box)
    :x_(x)
    ,y_(y)
    ,yaw_(yaw)
    ,xP_(0.)
    ,yP_(0.) //change to box.yMax_ + sightHorizon
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
// Updtating yaw coordinate
hessian_ = 0.;
hessian_ += xGetBackPredatorHessian(t);
gradient_ = 0.;
gradient_ += xGetBackPredatorGradient(t);
yaw_ += Solver::solve(hessian_, gradient_, wMax_, -wMax_)*t;

// Updating x and y coordinate
hessian_ = 0.;
hessian_ += xGetPredatorDistanceHessian(t);
gradient_ = 0.;
gradient_ += xGetPredatorDistanceHessian(t);
Scalar sol = Solver::solve(hessian_, gradient_, vMax_, 0.);
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

Scalar Agent::xGetBackPredatorHessian(Scalar t)
{
    return t*t;
}

Scalar Agent::xGetBackPredatorGradient(Scalar t)
{
    return t*(yaw_ - std::atan((y_- yP_)/(x_- xP_)));
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

Scalar Agent::xGetLowSpeedHessian(Scalar t)
{
    return 1.;
}

Scalar Agent::xGetLowSpeedGradient(Scalar t)
{
    return 0.;
}
