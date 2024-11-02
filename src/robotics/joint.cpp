#include "robotics/joint.h"

Joint::Joint(std::shared_ptr<Body> _Parent, std::shared_ptr<Body> _Child, Eigen::Matrix4d _TFrame, JointType _type)
            :mJointType(_type)
{
    mpParent = _Parent;
    mpChild = _Child;

    // set parent frame
    Eigen::Matrix4d T = _Parent->getT();
    mTParent2Joint = T.inverse() * _TFrame;

    // set child frame
    T = _Child->getT();
    mTChild2Joint = T.inverse() * _TFrame;
}

Eigen::Matrix4d Joint::getTransformation()
{
    Eigen::Matrix4d mat;
    mat << cos(mAngle), -sin(mAngle), 0, 0,
           sin(mAngle),  cos(mAngle), 0, 0,
                     0,            0, 1, 0,
                     0,            0, 0, 1;
    return mat;
}

void Joint::updateJoint(bool _IK)
{
    if(_IK || mJointType == JointType::PASSIVE)
    {
        Eigen::Matrix4d T = (mpParent.lock()->getT() * mTParent2Joint).inverse() * mpChild.lock()->getT() * mTChild2Joint;
        mAngle = atan2(T(1,0), T(0,0));
    }
}

void Joint::print()
{
    if(mJointType == JointType::ACTIVE)
    {
        std::cout << "Active Joint" << std::endl
                  << "Id: " << mnid << std::endl
                  << "Joint Angle: " << mAngle << std::endl;
    }
    else
    {
        std::cout << "Passive Joint" << std::endl
                  << "Id: " << mnid << std::endl
                  << "Joint Angle: " << mAngle << std::endl;
    }
    std::cout << "parent to center\n" << mTParent2Joint << std::endl;
    std::cout << "child to center\n" << mTChild2Joint << std::endl;
}