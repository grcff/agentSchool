#include "tools.h"
#include <cmath>



Scalar Solver::solve(Scalar a,
             Scalar b,
             Scalar xMax,
             Scalar xMin)
{
    assert (a != 0);
    Scalar xStat = -b/(2*a);

    if(a>0)
    {
        if(xStat > xMax)
        {
            return xMax;
        }
        else if(xStat < xMin)
        {
            return xMin;
        }
        else
        {
            return xStat;
        }
    }
    else if(a<0)
    {
        if(xStat > xMax)
        {
            return xMin;
        }
        else if(xStat < xMin)
        {
            return xMax;
        }
        else
        {
            return std::abs(xStat - xMax)>std::abs(xStat - xMin)?xMax:xMin;
        }

    }
}
