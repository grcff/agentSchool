#include "experience.h"

#include "src/types.cpp"
#include "src/predator.cpp"
#include "src/agent.cpp"

Experience::Experience(int nbAgent,
                       Scalar size,
                       Scalar xP,
                       Scalar yP,
                       Scalar v,
                       Scalar xMax,
                       Scalar yMax,
                       Scalar xMin,
                       Scalar yMin)
    :box_(xMax, yMax, xMin, yMin)
    ,predator_(xP, yP, v, box_)
    ,agentVec_(nbAgent,
               Agent(size,
                     v/1.5,
                     M_PI/60.,//M_PI/16.,
                     100,
                     M_PI/2.,
                     box_))
{
    xInit(nbAgent, v);
}
Experience::~Experience()
{}

Scalar Experience::getXP()
{
    return predator_.getX();
}
Scalar Experience::getYP()
{
    return predator_.getY();
}
Scalar Experience::getYawP()
{
    return predator_.getYaw();
}

Scalar Experience::getX(int i)
{
    return xVec_[i];
}

Scalar Experience::getY(int i)
{
    return yVec_[i];
}

Scalar Experience::getYaw(int i)
{
    return yawVec_[i];
}
//DEBUG:
size_t Experience::getNN(int i)
{
    return agentVec_[i].getXVec().size();
}

Scalar Experience::getXN(int i, int j)
{
    return agentVec_[i].getXVec()[j];
}

Scalar Experience::getYN(int i, int j)
{
    return agentVec_[i].getYVec()[j];
}

Scalar Experience::getYawN(int i, int j)
{
    return agentVec_[i].getYawVec()[j];
}


Scalar getYneig(int i, int j);
Scalar getYawNeig(int i, int j);

// TODO: delete ugly xVec and yVec
void Experience::update(Scalar t)
{
    predator_.updatePos(t);
    for (size_t i=0; i<agentVec_.size(); ++i)
    {
        xUpdateEnvironment(i);
        agentVec_[i].updatePos(t);
        xVec_[i] = agentVec_[i].getX();
        yVec_[i] = agentVec_[i].getY();
        yawVec_[i] = agentVec_[i].getYaw();
    }
}

void Experience::xInit(int nbAgent, Scalar v)
{
    // Seed rand function
    srand(time(NULL));

    xVec_.resize(nbAgent);
    yVec_.resize(nbAgent);
    yawVec_.resize(nbAgent);

    for(int i=0; i<nbAgent; ++i)
    {
        agentVec_[i].setX((box_.xMax_ - box_.xMin_)*((Scalar) rand() / (RAND_MAX)) + box_.xMin_);
        agentVec_[i].setY((box_.yMax_ - box_.yMin_)*((Scalar) rand() / (RAND_MAX)) + box_.yMin_);
        agentVec_[i].setYaw(2*M_PI*((Scalar) rand() / (RAND_MAX)));

        xVec_[i] = agentVec_[i].getX();
        yVec_[i] = agentVec_[i].getY();
        yawVec_[i] = agentVec_[i].getYaw();
    }
}


void Experience::xUpdateEnvironment(int agentIndex)
{
    Agent& agent(agentVec_[agentIndex]);

    agent.setXP(predator_.getX());
    agent.setYP(predator_.getY());

    agent.cleanNeigborVectors();

    Scalar diffX(0.);
    Scalar diffY(0.);

    for (size_t i=0; i<agentVec_.size(); ++i)
    {
        if(i!=agentIndex)
        {
            const Agent& neighbor(agentVec_[i]);
            diffX = neighbor.getX() - agent.getX();
            diffY = neighbor.getY() - agent.getY();


            // Here we fill the agent vector of neihbors
            // atan2 gives an angle between -Pi and Pi, agent's yaw angle is wrapped between 0 and 2*Pi
            // hence the wrapping of atan2
            if(std::abs(Tools::getClosestAngle(Tools::wrapAngle(std::atan2(diffY, diffX)), agent.getYaw())
                        - agent.getYaw()) < agent.getSightMaxAngle() &&
               std::abs(std::pow(diffX, 2) + std::pow(diffY, 2)) < std::pow(agent.getSightHorizon(), 2))
            {
                agent.addNeigbor(neighbor);
            }
        }
    }

}


#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(experience)
{
    //using namespace boost::python;
    //def("update", &Experience::update);

    class_<Experience>("Experience", init<int, Scalar, Scalar, Scalar, Scalar,
                       Scalar, Scalar, Scalar, Scalar>())
            .def("getXP", &Experience::getXP)
            .def("getYP", &Experience::getYP)
            .def("getYawP", &Experience::getYawP)
            .def("getX", &Experience::getX)
            .def("getY", &Experience::getY)
            .def("getYaw", &Experience::getYaw)
            .def("update", &Experience::update)
            //DEBUG:
            .def("getNN", &Experience::getNN)
            .def("getXN", &Experience::getXN)
            .def("getYN", &Experience::getYN)
            .def("getYawN", &Experience::getYawN)
            ;

};


