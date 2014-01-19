#include "predator.h"

Predator::Predator()
    :x_(0.)
    ,y_(0.)
    ,v_(0.)
    ,box_()
{}

Predator::Predator(Scalar x,
                   Scalar y,
                   Scalar v,
                   Box box)
    :x_(x)
    ,y_(y)
    ,v_(v)
    ,box_(box)
{
    assert(x < box_.xMax_ - EPSILON);
    assert(y < box_.yMax_ - EPSILON);
    assert(x > box_.xMin_ + EPSILON);
    assert(y > box_.yMin_ + EPSILON);
    assert(v > -EPSILON);

    yaw_ = 2*M_PI*((Scalar) rand() / (RAND_MAX));
}


Predator::~Predator()
{}


void Predator::updatePos(Scalar t)
{
    Scalar xNew = x_ + v_*t*std::cos(yaw_);
    Scalar yNew = y_ + v_*t*std::sin(yaw_);

    if(xNew > box_.xMax_ || xNew < box_.xMin_)
    {
        yaw_ = M_PI - yaw_;
        x_ = x_ + v_*t*std::cos(yaw_);
    }
    else
    {
        x_ = xNew;
    }

    if(yNew > box_.yMax_ || yNew < box_.yMin_)
    {
        yaw_ = -yaw_;
        y_ = y_ + v_*t*std::sin(yaw_);
    }
    else
    {
        y_ = yNew;
    }
}
