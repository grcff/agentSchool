#include <vector>

#include "src/types.h"
#include "src/predator.h"
#include "src/agent.h"

class Experience
{
public:
    Experience(int nbAgent,
               Scalar size,
               Scalar xP,
               Scalar yP,
               Scalar v,
               Scalar xMax,
               Scalar yMax,
               Scalar xMin,
               Scalar yMin);
    ~Experience();

    // TODO: tester avec const après

    Scalar getXP();
    Scalar getYP();
    Scalar getYawP();
    Scalar getX(int i);
    Scalar getY(int i);
    Scalar getYaw(int i);

    //DEBUG:
    size_t getNN(int i);
    Scalar getXN(int i, int j);
    Scalar getYN(int i, int j);
    Scalar getYawN(int i, int j);


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
