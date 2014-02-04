#ifndef AGENT_H
#define AGENT_H
#include <iostream>
#include "types.h"
#include "tools.h"

class Agent
{
public:
    // Constructors
    Agent();
    Agent(Scalar size,
          Scalar vMax,
          Scalar wMax,
          Scalar sightHorizon,
          Scalar sightMaxAngle, Box box);
    // Destructor
    ~Agent();

    // Update agent coordinates
    void updatePos(Scalar t);

    // Setters
    void setX(Scalar x);
    void setY(Scalar y);
    void setYaw(Scalar yaw);
    void addNeigbor(const Agent& agent);
    void setXP(Scalar xP);
    void setYP(Scalar yP);

    // Getters
    inline Scalar getX() const
    {return x_;}

    inline Scalar getY() const
    {return y_;}

    inline Scalar getYaw() const
    {return yaw_;}

    inline const VectorX& getXVec() const
    {return nXVec_;}

    inline const VectorX& getYVec() const
    {return nYVec_;}

    inline const VectorX& getYawVec() const
    {return nYawVec_;}

    inline const Scalar getVMax() const
    {return vMax_;}

    inline const Scalar getWMax() const
    {return wMax_;}

    inline const Scalar getSightHorizon() const
    {return sightHorizon_;}

    inline const Scalar getSightMaxAngle() const
    {return sightMaxAngle_;}


private:
    Scalar xGetMeanDirectionHessian(Scalar t);
    Scalar xGetMeanDirectionGradient(Scalar t);
    void xGetMeanDirectionWeight();

    Scalar xGetBackPredatorHessian(Scalar t);
    Scalar xGetBackPredatorGradient(Scalar t);
    void xGetBackPredatorWeight();

    Scalar xGetWanderAngleHessian(Scalar t);
    Scalar xGetWanderAngleGradient(Scalar t);


    Scalar xGetBarycenterHessian(Scalar t);
    Scalar xGetBarycenterGradient(Scalar t);
    void xGetBarycenterWeight();

    Scalar xGetPredatorDistanceHessian(Scalar t);
    Scalar xGetPredatorDistanceGradient(Scalar t);
    void xGetPredatorDistanceWeight();

    Scalar xGetLowSpeedHessian(Scalar t);
    Scalar xGetLowSpeedGradient(Scalar t);

    Scalar xGetWanderHessian(Scalar t);
    Scalar xGetWanderGradient(Scalar t);

    //Compute the maximum speed that the agent can reach
    Scalar xGetVMaxBound(Scalar t);

    // Agent coordinates and yaw angle
    Scalar x_;
    Scalar y_;
    Scalar yaw_;

    // Matrix of agent's neighbors coordinates
    // First column are neighbors's x coordinate
    // Second column are neighbors's y coordinate
    // Third column are neighbors's yaw coordinate
    VectorX nXVec_;
    VectorX nYVec_;
    VectorX nYawVec_;

    //Position of the predator
    Scalar xP_;
    Scalar yP_;

    const Scalar size_;
    const Scalar vMax_;
    const Scalar wMax_;
    const Scalar sightHorizon_;
    const Scalar sightMaxAngle_;

    Scalar hessian_;
    Scalar gradient_;

    Box box_;

    Scalar meanDirectionWeight_;
    Scalar backPredatorWeight_;
    Scalar barycenterWeight_;
    Scalar predatorDistanceWeight_;
};

#endif // AGENT_H
