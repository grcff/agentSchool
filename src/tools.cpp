#include "tools.h"
#include <cmath>



Scalar Tools::solve(Scalar a,
             Scalar b,
             Scalar xMax,
             Scalar xMin)
{
    assert (a != 0 || b == 0);

    if(a == 0 && b == 0)
    {
        return 0;
    }

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

Scalar Tools::wrapAngle(Scalar angle)
{
    return angle - 2*M_PI*std::floor(angle/(2*M_PI));
}

Scalar Tools::getClosestAngle(Scalar angle, Scalar reference)
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
