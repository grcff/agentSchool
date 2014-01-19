#ifndef PREDATOR_H
#define PREDATOR_H

#include <stdlib.h>
#include <cmath>
#include "types.h"

class Predator
{
public:
    Predator();
    Predator(Scalar x,
             Scalar y,
             Scalar v,
             Box box);

    ~Predator();

    // Update predator's position with respect to the
    void updatePos(Scalar t);

    inline Scalar getX() const
    {return x_;}
    inline Scalar getY() const
    {return y_;}
    inline Scalar getYaw() const
    {return yaw_;}

private:
    // Predator coordinates
    Scalar x_;
    Scalar y_;
    Scalar yaw_;

    // Predator speed
    Scalar v_;

    // The box in which the predator is allowed to move
    Box box_;
};

#endif // PREDATOR_H
