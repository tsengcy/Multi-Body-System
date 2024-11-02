#include "robotics/endeffector.h"

EndEffector::EndEffector(std::shared_ptr<Body> _pBody, Eigen::Matrix4d _T)
{
    mpBody = _pBody;
    mTBody2EE = mpBody.lock()->getT().inverse() * _T;
    std::cout << mTBody2EE << std::endl;

    mTGlobalEE = _T;
}

void EndEffector::update()
{
    mTGlobalEE = mpBody.lock()->getT() * mTBody2EE;
}

void EndEffector::setPose(Eigen::Matrix4d _T)
{
    mTGlobalEE = _T;
    mpBody.lock()->updatePose(_T * mTBody2EE.inverse());
    std::cout << _T * mTBody2EE.inverse() << std::endl;
}

void EndEffector::print()
{
    std::cout << "EndEffector\nId: " << mnid << std::endl
              << "Pose: \n" << mTGlobalEE << std::endl;
}



