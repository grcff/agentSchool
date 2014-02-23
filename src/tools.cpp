#include "tools.h"
#include <cmath>



Scalar Tools::solve(Scalar a,
                    Scalar b,
                    Scalar xMax,
                    Scalar xMin)
{
    Scalar xStat = -b/(2*a);

    if(a > EPSILON)
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
    else if(a < -EPSILON)
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
    else
    {
        if(std::abs(b) < EPSILON)
        {
            return 0.;
        }
        else
        {
            if(b > EPSILON)
            {
                return xMin;
            }
            else
            {
                return xMax;
            }
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

Scalar Tools::sigmoid(Scalar a, Scalar b, Scalar x)
{
    return 1/(1 + std::exp(-a*(x - b)));
}

