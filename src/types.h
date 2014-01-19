#pragma once
#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Core>

//Typedefs to abstract the choice between float and double
typedef double Scalar;

typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

static const Scalar EPSILON = 0.00001;

// Struct to define the space in which agents and predator are allowed to move
struct Box
{
    Box();
    Box(Scalar xMax,
        Scalar yMax,
        Scalar xMin,
        Scalar yMin);
    ~Box();

    Scalar xMax_;
    Scalar yMax_;
    Scalar xMin_;
    Scalar yMin_;
};

#endif // TYPES_H
