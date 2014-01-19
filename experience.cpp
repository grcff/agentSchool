#include "experience.h"

#include "src/types.cpp"
#include "src/predator.cpp"
#include "src/agent.cpp"

Experience::Experience(int nbAgent,
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
               Agent((box_.xMax_ - box_.xMin_)*((Scalar) rand() / (RAND_MAX)) + box_.xMin_,
                     (box_.yMax_ - box_.yMin_)*((Scalar) rand() / (RAND_MAX)) + box_.yMin_,
                     2*M_PI*((Scalar) rand() / (RAND_MAX)),
                     v,
                     M_PI/16.,
                     100,
                     3.*M_PI/4.,
                     box_))
//http://stackoverflow.com/questions/2860673/initializing-a-c-vector-to-random-values-fast
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

std::vector<Scalar> Experience::getXVec()
{
    return xVec_;
}

std::vector<Scalar> Experience::getYVec()
{
    return yVec_;
}

std::vector<Scalar> Experience::getYawVec()
{
    return yawVec_;
}

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
    xVec_.resize(nbAgent);
    yVec_.resize(nbAgent);
    yawVec_.resize(nbAgent);

    for(int i=0; i<nbAgent; ++i)
    {
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

    Scalar diffX(0.);
    Scalar diffY(0.);
    /*
    for (size_t i=0; i<agentVec_.size(); ++i)
    {
        if(i!=agentIndex)
        {
            Agent& neighbor(agentVec_[j]);
            diffX = neighbor.getX() - agent.getX();
            diffY = neighbor.getY() - agent.getY();

            //TODO: mettre autre chose que 30 (un attribut de agent)
            if(std::abs(std::atan(diffY)/diffX - agent.getYaw()) < 3.*M-PI/4. &&
                    std::abs(diffX*diffX + diffY*diffY) < 30.)
            {
                int agent.getNVec().
                        agent.setNVecSize()
            }


        }

    }
*/
}

#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(experience)
{
    //using namespace boost::python;
    //def("update", &Experience::update);

    class_<Experience>("Experience", init<int, Scalar, Scalar, Scalar,
                       Scalar, Scalar, Scalar, Scalar>())
            .def("getXP", &Experience::getXP)
            .def("getYP", &Experience::getYP)
            .def("getYawP", &Experience::getYawP)
            .def("getXVec", &Experience::getXVec)
            .def("getYVec", &Experience::getYVec)
            .def("getYawVec", &Experience::getYawVec)
            .def("update", &Experience::update)
            ;

};


