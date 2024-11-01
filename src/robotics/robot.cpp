#include <robotics/robot.h>
#include <stdexcept>
#include <sophus/so3.hpp>
#include "robotics/mathfunction.h"

Robot::Robot()
{

}

void Robot::insertBody(Eigen::Matrix4d _T, bool _bFixed)
{
    std::shared_ptr<Body> pBody(new Body(_T, _bFixed));
    mvpBody.push_back(pBody);
    pBody->setId(mvpBody.size()-1);
}

void Robot::insertJoint(Eigen::Matrix4d _T, int _nParentId, int _nChildId, JointType _type)
{
    if(_nParentId >= mvpBody.size())
    {
        std::stringstream ss;
        ss << "Body " << _nParentId << " is not a body, which is out of the range of the vector of Bodys\n";
        std::invalid_argument(ss.str());
    }

    if(_nChildId >= mvpBody.size())
    {
        std::stringstream ss;
        ss << "Body " << _nChildId << " is not a body, which is out of the range of the vector of Bodys\n";
        std::invalid_argument(ss.str());
    }

    std::shared_ptr<Joint> pJoint(new Joint(mvpBody[_nParentId], mvpBody[_nChildId], _T, _type));
    if(_type == JointType::ACTIVE)
    {
        mvpActiveJoint.push_back(pJoint);
        pJoint->setId(mvpActiveJoint.size()-1);
    }
    else
    {
        mvpPassiveJoint.push_back(pJoint);
        pJoint->setId(mvpPassiveJoint.size()-1);
    }
}

void Robot::setActiveJointAngle(std::vector<double> _angle)
{
    if(_angle.size() != mvpActiveJoint.size())
    {
        std::stringstream ss;
        ss << "[ERROR]: " << __FILE__ << " Line: " << __LINE__ 
           << "\nwrong size of vector\n" 
           << "the size of _angle is " << _angle.size() << ", expect " << mvpActiveJoint.size() << "\n";
        std::invalid_argument(ss.str());
    }

    for(int i=0; i<mvpActiveJoint.size(); i++)
    {
        mvpActiveJoint[i]->setAngle(_angle[i]);
    }
}

#define MAXITER 4000

void Robot::kinematics()
{
    int n = mvpBody.size();
    // int n = mvNoneFixedId.size();
    int m = mvpActiveJoint.size() + mvpPassiveJoint.size();

    // set all body's pose to se3 vector
    Eigen::MatrixXd vT = Eigen::MatrixXd::Zero(4*n, 4);
    for(int i=0; i<mvpBody.size(); i++)
    {
        vT.block<4, 4>(4*i, 0) = mvpBody[i]->getT();
    }
    
    
    // every 6 equation is the constraint
    // the front 3 is the center
    // for active the lst 3 is the x axis 
    // for passive the last 3 is the z axis
    int iter = 0;
    for(; iter < MAXITER; iter++)
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6*m, 6*n);
        Eigen::VectorXd dp = Eigen::VectorXd::Zero(6*m);
        int JointId = 0;
        for(int i=0; i<mvpPassiveJoint.size(); i++, JointId++)
        {
            std::shared_ptr<Joint> pJoint = mvpPassiveJoint[i];
            // parent
            int id = pJoint->getParentId();

            Eigen::Matrix4d T = vT.block<4,4>(4*id, 0);
            T = T * pJoint->getTParent2Joint();
            dp.block<3,1>(JointId*6, 0) = T.block<3,1>(0,3);

            if(!pJoint->getParent()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getParentId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getParentId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getParentId() * 6 + 3) = -1 * Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            
            Eigen::Vector4d pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) = pz.block<3,1>(0,0);
            
            if(!pJoint->getParent()->isFixed())
                J.block<3,3>(JointId*6+3, pJoint->getParentId() * 6 + 3) = -1 * Sophus::SO3d::hat(pz.block<3,1>(0,0));

            // child
            id = pJoint->getChildId();
            T = vT.block<4,4>(4*id, 0);
            T = T * pJoint->getTChild2Joint();
            dp.block<3,1>(JointId*6, 0) -= T.block<3,1>(0,3);

            if(!pJoint->getChild()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getChildId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getChildId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getChildId() * 6 + 3) = Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) -= pz.block<3,1>(0,0);
            
            if(!pJoint->getChild()->isFixed())
                J.block<3,3>(JointId*6+3, pJoint->getChildId() * 6 + 3) = Sophus::SO3d::hat(pz.block<3,1>(0,0));
        }

        for(int i=0; i<mvpActiveJoint.size(); i++, JointId++)
        {
            std::shared_ptr<Joint> pJoint = mvpActiveJoint[i];
            // parent
            int id = pJoint->getParentId();
            Eigen::Matrix4d T = vT.block<4,4>(4*id, 0);
            T = T * pJoint->getTParent2Joint();
            dp.block<3,1>(JointId*6, 0) = T.block<3,1>(0,3);

            if(!pJoint->getParent()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getParentId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getParentId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getParentId() * 6 + 3) = -1 * Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            
            Eigen::Vector4d px = T * pJoint->getTransformation() * Eigen::Vector4d(1, 0, 0, 1);
            dp.block<3,1>(JointId*6+3, 0) = px.block<3,1>(0,0);
            // std::cout << "px: ";
            // for(int i=0; i<3; i++)
            // {
            //     std::cout << px(i) << "\t";
            // }
            // std::cout << std::endl;
            
            if(!pJoint->getParent()->isFixed())
                J.block<3,3>(JointId*6+3, pJoint->getParentId() * 6 + 3) = -1 * Sophus::SO3d::hat(px.block<3,1>(0,0));

            // child
            id = pJoint->getChildId();
            T = vT.block<4,4>(4*id, 0);
            T = T * pJoint->getTChild2Joint();
            dp.block<3,1>(JointId*6, 0) -= T.block<3,1>(0,3);
            if(!pJoint->getChild()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getChildId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getChildId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getChildId() * 6 + 3) = Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            px = T * Eigen::Vector4d(1, 0, 0, 1);
            // std::cout << "px: ";
            // for(int i=0; i<3; i++)
            // {
            //     std::cout << px(i) << "\t";
            // }
            // std::cout << std::endl;
            dp.block<3,1>(JointId*6+3, 0) -= px.block<3,1>(0,0);
            
            if(!pJoint->getChild()->isFixed())
                J.block<3,3>(JointId*6+3, pJoint->getChildId() * 6 + 3) = Sophus::SO3d::hat(px.block<3,1>(0,0));
        }

        // std::cout << J << std::endl << "---\n" << dp << std::endl;
        if(dp.norm() < 0.001)
        {
            break;
        }
        Eigen::VectorXd dq = mathfunction::pseudoInverse(J) * -1 * dp;

        for(int i=0; i<mvpBody.size(); i++)
        {
            vT.block<4,4>(4*i, 0) = Sophus::SE3d::exp(dq.block<6,1>(6*i, 0)).matrix() * vT.block<4,4>(4*i, 0);
        }
    }
    if(iter == MAXITER)
    {
        std::cout << "unable to solve " << MAXITER << std::endl;
    }
    else
    {
        std::cout << "solve with " << iter+1 << std::endl;
    }

    for(int i=0; i<mvpBody.size(); i++)
    {
        mvpBody[i]->updatePose(vT.block<4,4>(4*i, 0));
    }
}

void Robot::print(bool _body, bool _joint)
{
    std::cout << "----------------------\n";
    if(_body)
    {
        for(int i=0; i<mvpBody.size(); i++)
        {
            mvpBody[i]->print();
            std::cout << "----------------------\n";
        }
    }

    if(_joint)
    {
        for(int i=0; i<mvpActiveJoint.size(); i++)
        {
            mvpActiveJoint[i]->print();
            std::cout << "----------------------\n";
        }
        for(int i=0; i<mvpPassiveJoint.size(); i++)
        {
            mvpPassiveJoint[i]->print();
            std::cout << "----------------------\n";
        }
    }
}




