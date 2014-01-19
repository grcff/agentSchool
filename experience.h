#include <vector>

#include "src/types.h"
#include "src/predator.h"
#include "src/agent.h"

class Experience
{
public:
    Experience(int nbAgent,
               Scalar xP,
               Scalar yP,
               Scalar v,
               Scalar xMax,
               Scalar yMax,
               Scalar xMin,
               Scalar yMin);
    ~Experience();

    //tester avec const après

    Scalar getXP();
    Scalar getYP();
    Scalar getYawP();
    std::vector<Scalar> getXVec();
    std::vector<Scalar> getYVec();
    std::vector<Scalar> getYawVec();

    void update(Scalar t);

private:
    void xInit(int nbAgent, Scalar v);
    void xUpdateEnvironment(int agentIndex);

private:
    Box box_;
    Predator predator_;
    std::vector<Agent> agentVec_;

    std::vector<Scalar> xVec_;
    std::vector<Scalar> yVec_;
    std::vector<Scalar> yawVec_;
};
