#include "robotics/body.h"

Body::Body(Eigen::Matrix4d _TFrame, bool _bFixed)
{
    mTFrame = _TFrame;
    mbFixed = _bFixed;
}

// void Body::insertJoint(std::shared_ptr<Joint> _pJoint, Eigen::Matrix4d _TJointCenter)
// {
//     mvpJoint.push_back(_pJoint);
//     mvTJointCenter.push_back(_TJointCenter);
// }

void Body::setId(int _id)
{
    mnid = _id;
}

void Body::print()
{
    std::cout << "Body Frame\n" << "Id: " << mnid << "\nPose:\n" << mTFrame 
              << "\nisFixed: " << (mbFixed ? "TRUE" : "FALSE") << std::endl;
}

