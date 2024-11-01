#ifndef _MATHFUNCTION_H__
#define _MATHFUNCTION_H__
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace mathfunction
{
    Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd mat);
};

#endif // _MATHFUNCTION_H__
