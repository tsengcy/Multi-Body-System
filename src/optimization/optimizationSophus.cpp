#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#define EPOSILON 1E-4

Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd mat)
{
    int row = mat.rows();
    int col = mat.cols();

    if(row > col)
    {
        float manipulability = (mat.transpose() * mat).determinant();
        // std::cout << "----\n" << (mat.transpose() * mat) << "\n--" << std::endl;
        if(manipulability < EPOSILON)
        {
            std::cout << "aa" << std::endl;
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
            std::cout << "bb" << std::endl;
            return mat.transpose() * (mat * mat.transpose() + Eigen::MatrixXd::Identity(row, row) * EPOSILON).inverse();
        }
        else
        {
            return mat.transpose() * (mat * mat.transpose()).inverse();
        }
    }
}

int main()
{
    std::vector<Eigen::Vector4d> vp;
    vp.push_back(Eigen::Vector4d(1, 0, 0, 1));
    vp.push_back(Eigen::Vector4d(0, 1, 0, 1));
    vp.push_back(Eigen::Vector4d(0, 0, 1, 1));

    std::vector<Eigen::Vector3d> vans;
    vans.push_back(Eigen::Vector3d(-1, 1,  0));
    vans.push_back(Eigen::Vector3d( 0, 2,  0));
    vans.push_back(Eigen::Vector3d( 0, 1, -1));

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    double error = 0;
    int m = vans.size();
    for(int iter=0; iter < 400; iter++)
    {
        Sophus::SE3d T = Sophus::SE3d::exp(q);
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3*m, 6);
        Eigen::VectorXd dp = Eigen::VectorXd::Zero(3*m);
        for(int i=0; i<vp.size(); i++)
        {
            Eigen::Vector3d p = T.matrix3x4() * vp[i];
            dp.block<3,1>(3*i, 0) = vans[i] - p;
            J.block<3,3>(3*i, 0) = Eigen::Matrix3d::Identity();
            J.block<3,3>(3*i, 3) = -1 * Sophus::SO3d::hat(p);
        }

        error = dp.norm();
        if(error < 0.0001)
            break;
        Eigen::VectorXd dq = pseudoInverse(J) * dp;
        T = Sophus::SE3d::exp(dq) * T;
        q = T.log();
        std::cout << "iter: " << iter << " ";
        for(int i=0; i<6; i++)
        {
            std::cout << q(i) << "\t";
        }
        std::cout << std::endl;
    }

    // for(int i=0; i<6; i++)
    // {
    //     std::cout << q(i) << "\t";
    // }
    // std::cout << std::endl;

    std::cout << "matrix" << std::endl;
    std::cout << Sophus::SE3d::exp(q).matrix() << std::endl;

}