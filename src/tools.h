#ifndef TOOLS_H
#define TOOLS_H

#include "types.h"

namespace Tools
{
// Find x in order to minimize ax^2+bx under the constraint xMin < x < xMax
static Scalar solve(Scalar a,
                    Scalar b,
                    Scalar xMax,
                    Scalar xMin);

// Wrap angle between 0 and 2Pi
static Scalar wrapAngle(Scalar angle);

// Add or deduct 2Pi from angle so that it is the closest value to reference
static Scalar getClosestAngle(Scalar angle, Scalar reference);

//Compute the value in x of a sigmoid of parameter a and centered on x=b
static Scalar sigmoid(Scalar a, Scalar b, Scalar x);
}
#endif // TOOLS_H
