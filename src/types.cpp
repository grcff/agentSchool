#include "types.h"

Box::Box()
    :xMax_(0.)
    ,yMax_(0.)
    ,xMin_(0.)
    ,yMin_(0.)
{}
Box::Box(Scalar xMax,
         Scalar yMax,
         Scalar xMin,
         Scalar yMin)
    :xMax_(xMax)
    ,yMax_(yMax)
    ,xMin_(xMin)
    ,yMin_(yMin)
{}

Box::~Box()
{}
