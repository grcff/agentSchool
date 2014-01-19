#ifndef TOOLS_H
#define TOOLS_H

#include "types.h"

struct Solver
{
// Find x in order to minimize ax^2+bx under the constraint xMax < x < xMin
static Scalar solve(Scalar a,
                    Scalar b,
                    Scalar xMax,
                    Scalar xMin);
};
#endif // TOOLS_H
