#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <stdexcept>

Eigen::Matrix3d skew(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mat;
    mat <<       0,   -vec(2),    vec(1),
            vec(2),         0,   -vec(0),
           -vec(1),    vec(0),         0;
    return mat;
}

Eigen::Matrix3d Rodrigues(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
    double angle = vec.norm();
    if(angle < 0.0001)
    {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d axis = vec / angle;
    mat += sin(angle) * skew(axis) + (1 - cos(angle)) * skew(axis) * skew(axis);

    return mat;
}

Eigen::Matrix3d derivativeOfRotation(Eigen::Vector3d vec, Eigen::Vector3d p)
{
    Eigen::Matrix3d mat;
    Eigen::Matrix3d tmp;
    double angle = vec.norm();
    Eigen::Vector3d axis = vec / angle;

    tmp << 2 * sin(angle) * axis(0),      sin(angle) * axis(1),      sin(angle) * axis(2),
               sin(angle) * axis(1), -2 * sin(angle) * axis(0),               -cos(angle),
               sin(angle) * axis(2),                cos(angle), -2 * sin(angle) * axis(0);

    mat.block<3,1>(0,0) = tmp * p;

    tmp << -2 * sin(angle) * axis(1),     sin(angle) * axis(0),                cos(angle),
                sin(angle) * axis(0), 2 * sin(angle) * axis(1),      sin(angle) * axis(2),
                         -cos(angle),     sin(angle) * axis(2), -2 * sin(angle) * axis(1);

    mat.block<3,1>(0,1) = tmp * p;

    tmp << -2 * sin(angle) * axis(2),               -cos(angle),     sin(angle) * axis(0),
                          cos(angle), -2 * sin(angle) * axis(2),     sin(angle) * axis(1),
                sin(angle) * axis(0),      sin(angle) * axis(1), 2 * sin(angle) * axis(2);

    mat.block<3,1>(0,2) = tmp * p;

    return mat;
}


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

bool solve(Eigen::VectorXd& vec, std::vector<Eigen::Vector3d> vp, std::vector<Eigen::Vector3d> vans)
{
    if(vp.size() != vans.size())
    {
        std::stringstream ss;
        ss << "error of array size at file: " << __FILE__ << " line: " << __LINE__ << "\n";
        std::invalid_argument(ss.str());
    }
    int m = vp.size();
    int n = vec.rows();
    Eigen::Matrix3d R = Rodrigues(vec.block<3,1>(0,0));

    Eigen::MatrixXd J(3*m, n);
    Eigen::VectorXd dp = Eigen::VectorXd::Zero(3*m);
    Eigen::VectorXd p = Eigen::VectorXd::Zero(3*m);
    for(int i=0; i<vans.size(); i++)
    {
        p.block<3,1>(3*i, 0) = R * vp[i] + vec.block<3,1>(3,0);
        dp.block<3,1>(3*i,0) = R * vp[i] + vec.block<3,1>(3,0) - vans[i];
        J.block<3,3>(3*i, 3) = Eigen::Matrix3d::Identity();
        J.block<3,3>(3*i, 0) = derivativeOfRotation(vec.block<3,1>(0,0), vp[i]);
    }
    // std::cout << "p: ";
    // for(int i=0; i<3*m; i++)
    // {
    //     std::cout << p(i) << "\t";
    // }
    // std::cout << std::endl;
    // std::cout << "dp: " << dp.norm() << std::endl;
    if(dp.norm() < 0.01)
    {
        return true;
    }

    vec = vec - pseudoInverse(J) * dp;
    double angle = vec.block<3,1>(0,0).norm();
    
    if(angle < 0.001)
    {
        // std::cout << "angle: " << angle << std::endl;
        return false;
    }   
    
    int na = angle / (2*M_PI);
    // std::cout << "angle: " << angle << "\tna: " << na << std::endl;
    vec.block<3,1>(0,0) = vec.block<3,1>(0,0) / angle * (angle - na * 2*M_PI);
    return false;
}

int main()
{
    std::vector<Eigen::Vector3d> vp;
    vp.push_back(Eigen::Vector3d(0, 1, 0));
    vp.push_back(Eigen::Vector3d(1, 0, 0));
    vp.push_back(Eigen::Vector3d(0, 0, 1));

    std::vector<Eigen::Vector3d> vans;
    vans.push_back(Eigen::Vector3d( 0, 2,  0));
    vans.push_back(Eigen::Vector3d(-1, 1,  0));
    vans.push_back(Eigen::Vector3d( 0, 1, -1));

    Eigen::VectorXd vec(6);
    vec << 0, 1, 0, 0, 0, 0;
    int i=0;
    for(; i<4000; i++)
    {
        if(solve(vec, vp, vans))
        {
            break;
        }
        // std::cout << "i: " << i << "\t";
        // for(int j=0; j<6; j++)
        // {
        //     std::cout << vec(j) << "\t";
        // }
        // std::cout << std::endl;
    }
    std::cout << i << std::endl;

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3,3>(0,0) = Rodrigues(vec.block<3,1>(0,0));
    mat.block<3,1>(0,3) = vec.block<3,1>(3,0);
    std::cout << "matrix\n" << mat << std::endl;
}