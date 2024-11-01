#include "robotics/mathfunction.h"

#define EPOSILON 1E-4

Eigen::MatrixXd mathfunction::pseudoInverse(Eigen::MatrixXd mat)
{
    int row = mat.rows();
    int col = mat.cols();

    if(row > col)
    {
        float manipulability = (mat.transpose() * mat).determinant();
        // std::cout << "----\n" << (mat.transpose() * mat) << "\n--" << std::endl;
        if(manipulability < EPOSILON)
        {
            // std::cout << "----\n" << (mat.transpose() * mat + Eigen::MatrixXf::Identity(col, col) * EPOSILON).inverse() << "\n--" << std::endl;
            return (mat.transpose() * mat + Eigen::MatrixXd::Identity(col, col) * EPOSILON).inverse() * mat.transpose();
        }
        else
        {
            return (mat.transpose() * mat).inverse() * mat.transpose();
        }
    }
    else
    {
        float manipulability = (mat * mat.transpose()).determinant();
        if(manipulability < EPOSILON)
        {
            return mat.transpose() * (mat * mat.transpose() + Eigen::MatrixXd::Identity(row, row) * EPOSILON).inverse();
        }
        else
        {
            return mat.transpose() * (mat * mat.transpose()).inverse();
        }
    }
}