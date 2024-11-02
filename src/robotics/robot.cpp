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

    if(_bFixed)
    {
        pBody->setId(mvpBody.size()-1, mnFixedBody++);
    }
    else
    {
        pBody->setId(mvpBody.size()-1, mnMoveBody++);
    }
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

void Robot::insertEndEffector(Eigen::Matrix4d _T, int _nParentId)
{
    if(_nParentId >= mvpBody.size())
    {
        std::stringstream ss;
        ss << "Body " << _nParentId << " is not a body, which is out of the range of the vector of Bodys\n";
        std::invalid_argument(ss.str());
    }
    std::shared_ptr<EndEffector> pEE(new EndEffector(mvpBody[_nParentId], _T));
    mvpEndEffector.push_back(pEE);
    pEE->setId(mvpEndEffector.size()-1);
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

bool Robot::ForwardKinematics()
{
    int m = mvpActiveJoint.size() + mvpPassiveJoint.size();

    // set all body's pose to se3 vector
    Eigen::MatrixXd vTFixed = Eigen::MatrixXd::Zero(4*mnFixedBody, 4);
    Eigen::MatrixXd vTMoved = Eigen::MatrixXd::Zero(4*mnMoveBody, 4);

    int iMove = 0, iFixed = 0;
    for(int i=0; i<mvpBody.size(); i++)
    {
        if(mvpBody[i]->isFixed())
        {
            vTFixed.block<4,4>(4*iFixed, 0) = mvpBody[i]->getT();
            iFixed++;
        }
        else
        {
            vTMoved.block<4,4>(4*iMove, 0) = mvpBody[i]->getT();
            iMove++;
        }
    }

    // every 6 equation is the constraint
    // the front 3 is the center
    // for active the lst 3 is the x axis 
    // for passive the last 3 is the z axis
    int iter = 0;
    for(; iter < MAXITER; iter++)
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6*m, 6*mnMoveBody);
        Eigen::VectorXd dp = Eigen::VectorXd::Zero(6*m);
        int JointId = 0;
        for(int i=0; i<mvpPassiveJoint.size(); i++, JointId++)
        {
            std::shared_ptr<Joint> pJoint = mvpPassiveJoint[i];

            // parent
            Eigen::Matrix4d T;
            if(pJoint->getParent()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getParentMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getParentMoveId(), 0);

            T = T * pJoint->getTParent2Joint();
            dp.block<3,1>(JointId*6, 0) = T.block<3,1>(0,3);

            if(!pJoint->getParent()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            
            Eigen::Vector4d pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) = pz.block<3,1>(0,0);
            
            if(!pJoint->getParent()->isFixed())
                J.block<3,3>(JointId*6+3, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(pz.block<3,1>(0,0));

            // child
            if(pJoint->getChild()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getChildMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getChildMoveId(), 0);

            T = T * pJoint->getTChild2Joint();
            dp.block<3,1>(JointId*6, 0) -= T.block<3,1>(0,3);

            if(!pJoint->getChild()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) -= pz.block<3,1>(0,0);
            
            if(!pJoint->getChild()->isFixed())
                J.block<3,3>(JointId*6+3, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(pz.block<3,1>(0,0));
        }

        for(int i=0; i<mvpActiveJoint.size(); i++, JointId++)
        {
            std::shared_ptr<Joint> pJoint = mvpActiveJoint[i];

            // parent
            Eigen::Matrix4d T;
            if(pJoint->getParent()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getParentMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getParentMoveId(), 0);

            T = T * pJoint->getTParent2Joint();
            dp.block<3,1>(JointId*6, 0) = T.block<3,1>(0,3);

            if(!pJoint->getParent()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(T.block<3,1>(0,3));
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
                J.block<3,3>(JointId*6+3, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(px.block<3,1>(0,0));

            // child
            if(pJoint->getChild()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getChildMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getChildMoveId(), 0);

            T = T * pJoint->getTChild2Joint();
            dp.block<3,1>(JointId*6, 0) -= T.block<3,1>(0,3);
            if(!pJoint->getChild()->isFixed())
            {
                J.block<3,3>(  JointId*6,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(T.block<3,1>(0,3));
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
                J.block<3,3>(JointId*6+3, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(px.block<3,1>(0,0));
        }

        // std::cout << J << std::endl << "---\n" << dp << std::endl;
        if(dp.norm() < 0.001)
        {
            break;
        }
        Eigen::VectorXd dq = mathfunction::pseudoInverse(J) * -1 * dp;

        iMove = 0;
        for(int i=0; i<mvpBody.size(); i++)
        {
            if(!mvpBody[i]->isFixed())
            {
                vTMoved.block<4,4>(4*iMove, 0) = Sophus::SE3d::exp(dq.block<6,1>(6*iMove, 0)).matrix() * vTMoved.block<4,4>(4*iMove, 0);
                iMove++;
            }
        }
    }

    iMove = 0;
    for(int i=0; i<mvpBody.size(); i++)
    {
        if(!mvpBody[i]->isFixed())
        {
            mvpBody[i]->updatePose(vTMoved.block<4,4>(4*iMove, 0));
            iMove++;
        }
    }

    for(int i=0; i<mvpPassiveJoint.size(); i++)
    {
        mvpPassiveJoint[i]->updateJoint();
    }

    for(auto &ee : mvpEndEffector)
    {
        ee->update();
    }

    if(iter == MAXITER)
    {
        std::cout << "unable to solve " << MAXITER << std::endl;
        return false;
    }
    else
    {
        std::cout << "solve with " << iter+1 << std::endl;
        return true;
    }
}

bool Robot::InverseKinematics(std::vector<int> _vEEId, std::vector<Eigen::Matrix4d> _vEEFrame)
{
    if(_vEEFrame.size() != _vEEId.size())
    {
        std::stringstream ss;
        ss << "[ERROR]: " << __FILE__ << "Line: " << __LINE__
           << "\nInverse Kinematics input size ids and Frames are not equal" << std::endl;
        std::invalid_argument(ss.str());
    }

    /*
    1. set the give ee pose to relative body
    2. check the constraint for the joint that with two end have set Frame value or fixed
    3. use Jacobain to update othe body
    */
    for(auto& ee : mvpEndEffector)
    {
        ee->resetIKMode();
    }

    for(int i=0; i<_vEEId.size(); i++)
    {
        if(_vEEId[i] >= mvpEndEffector.size())
        {
            std::stringstream ss;
            ss << "[ERROR]: " << __FILE__ << "Line: " << __LINE__
               << "\nwrong id of endeffector" << std::endl;
            std::invalid_argument(ss.str());
        }
        else
        {
            mvpEndEffector[_vEEId[i]]->setPose(_vEEFrame[i]);
            mvpEndEffector[_vEEId[i]]->setIKMode(true);
        }
    }

    int m = 0;

    for(int i=0; i<mvpPassiveJoint.size(); i++)
    {
        std::shared_ptr<Joint> pJoint = mvpPassiveJoint[i];
        if(pJoint->getParent()->getIKMode() && pJoint->getChild()->getIKMode())
        {
            Eigen::Matrix4d Tparent = pJoint->getParent()->getT() * pJoint->getTParent2Joint();
            Eigen::Matrix4d Tchild = pJoint->getChild()->getT() * pJoint->getTChild2Joint();
            Eigen::Matrix4d Tdif = Tparent.inverse() * Tchild;
            Eigen::AngleAxisd aa(Tdif.block<3,3>(0,0));
            if(aa.angle() > 0.01 || Tdif.block<3,1>(0,0).norm() > 0.001)
            {
                std::stringstream ss;
                ss << "[ERROR]: " __FILE__ << "Line: " << __LINE__
                   << "\n The given value of Body fail to satisfy the constraint of the passive joint " << pJoint->getId() << std::endl;
                std::invalid_argument(ss.str()); 
            }
        }
        else
            m++;
    }

    for(int i=0; i<mvpActiveJoint.size(); i++)
    {
        std::shared_ptr<Joint> pJoint = mvpActiveJoint[i];
        if(pJoint->getParent()->getIKMode() && pJoint->getChild()->getIKMode())
        {
            Eigen::Matrix4d Tparent = pJoint->getParent()->getT() * pJoint->getTParent2Joint();
            Eigen::Matrix4d Tchild = pJoint->getChild()->getT() * pJoint->getTChild2Joint();
            Eigen::Matrix4d Tdif = Tparent.inverse() * Tchild;
            Eigen::AngleAxisd aa(Tdif.block<3,3>(0,0));
            if(aa.angle() > 0.01 || Tdif.block<3,1>(0,0).norm() > 0.001)
            {
                std::stringstream ss;
                ss << "[ERROR]: " __FILE__ << "Line: " << __LINE__
                   << "\n The given value of Body fail to satisfy the constraint of the Active joint " << pJoint->getId() << std::endl;
                std::invalid_argument(ss.str()); 
            }
        }
        else
            m++;
    }

    std::cout << "m: " << m << std::endl;


    Eigen::MatrixXd vTFixed = Eigen::MatrixXd::Zero(4*mnFixedBody, 4);
    Eigen::MatrixXd vTMoved = Eigen::MatrixXd::Zero(4*mnMoveBody, 4);

    int iMove = 0, iFixed = 0;
    for(int i=0; i<mvpBody.size(); i++)
    {
        if(mvpBody[i]->isFixed())
        {
            vTFixed.block<4,4>(4*iFixed, 0) = mvpBody[i]->getT();
            iFixed++;
        }
        else
        {
            vTMoved.block<4,4>(4*iMove, 0) = mvpBody[i]->getT();
            iMove++;
        }
    }

    // every 6 equation is the constraint
    // the front 3 is the center
    // for active the lst 3 is the x axis 
    // for passive the last 3 is the z axis
    int iter = 0;
    for(; iter < MAXITER; iter++)
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6*m, 6*mnMoveBody);
        Eigen::VectorXd dp = Eigen::VectorXd::Zero(6*m);
        int JointId = 0;
        for(int i=0; i<mvpPassiveJoint.size(); i++)
        {
            std::shared_ptr<Joint> pJoint = mvpPassiveJoint[i];
            if(pJoint->getParent()->getIKMode() && pJoint->getChild()->getIKMode())
                continue;

            // parent
            Eigen::Matrix4d T;
            if(pJoint->getParent()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getParentMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getParentMoveId(), 0);

            T = T * pJoint->getTParent2Joint();
            dp.block<3,1>(JointId*6, 0) = T.block<3,1>(0,3);

            if(!pJoint->getParent()->getIKMode())
            {
                J.block<3,3>(  JointId*6,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            
            Eigen::Vector4d pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) = pz.block<3,1>(0,0);
            
            if(!pJoint->getParent()->getIKMode())
                J.block<3,3>(JointId*6+3, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(pz.block<3,1>(0,0));

            // child
            if(pJoint->getChild()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getChildMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getChildMoveId(), 0);

            T = T * pJoint->getTChild2Joint();
            dp.block<3,1>(JointId*6, 0) -= T.block<3,1>(0,3);

            if(!pJoint->getChild()->getIKMode())
            {
                J.block<3,3>(  JointId*6,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) -= pz.block<3,1>(0,0);
            
            if(!pJoint->getChild()->getIKMode())
                J.block<3,3>(JointId*6+3, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(pz.block<3,1>(0,0));
            JointId++;
        }

        for(int i=0; i<mvpActiveJoint.size(); i++)
        {
            std::shared_ptr<Joint> pJoint = mvpActiveJoint[i];
            if(pJoint->getParent()->getIKMode() && pJoint->getChild()->getIKMode())
                continue;

            // parent
            Eigen::Matrix4d T;
            if(pJoint->getParent()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getParentMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getParentMoveId(), 0);

            T = T * pJoint->getTParent2Joint();
            dp.block<3,1>(JointId*6, 0) = T.block<3,1>(0,3);

            if(!pJoint->getParent()->getIKMode())
            {
                J.block<3,3>(  JointId*6,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getParentMoveId() * 6) = Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            
            Eigen::Vector4d pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) = pz.block<3,1>(0,0);
            
            if(!pJoint->getParent()->getIKMode())
                J.block<3,3>(JointId*6+3, pJoint->getParentMoveId() * 6 + 3) = -1 * Sophus::SO3d::hat(pz.block<3,1>(0,0));

            // child
            if(pJoint->getChild()->isFixed())
                T = vTFixed.block<4,4>(4*pJoint->getChildMoveId(), 0);
            else
                T = vTMoved.block<4,4>(4*pJoint->getChildMoveId(), 0);

            T = T * pJoint->getTChild2Joint();
            dp.block<3,1>(JointId*6, 0) -= T.block<3,1>(0,3);

            if(!pJoint->getChild()->getIKMode())
            {
                J.block<3,3>(  JointId*6,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(JointId*6+3,     pJoint->getChildMoveId() * 6) = -1 * Eigen::Matrix3d::Identity();
                J.block<3,3>(  JointId*6, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(T.block<3,1>(0,3));
            }
            pz = T * Eigen::Vector4d(0, 0, 1, 1);
            dp.block<3,1>(JointId*6+3, 0) -= pz.block<3,1>(0,0);
            
            if(!pJoint->getChild()->getIKMode())
                J.block<3,3>(JointId*6+3, pJoint->getChildMoveId() * 6 + 3) = Sophus::SO3d::hat(pz.block<3,1>(0,0));
            JointId++;
        }

        // std::cout << J << std::endl << "---\n" << dp << std::endl;
        if(dp.norm() < 0.001)
        {
            break;
        }
        Eigen::VectorXd dq = mathfunction::pseudoInverse(J) * -1 * dp;

        iMove = 0;
        for(int i=0; i<mvpBody.size(); i++)
        {
            if(!mvpBody[i]->isFixed())
            {
                vTMoved.block<4,4>(4*iMove, 0) = Sophus::SE3d::exp(dq.block<6,1>(6*iMove, 0)).matrix() * vTMoved.block<4,4>(4*iMove, 0);
                iMove++;
            }
        }
    }

    iMove = 0;
    for(int i=0; i<mvpBody.size(); i++)
    {
        if(!mvpBody[i]->getIKMode())
        {
            mvpBody[i]->updatePose(vTMoved.block<4,4>(4*iMove, 0));
            iMove++;
        }
    }

    for(int i=0; i<mvpPassiveJoint.size(); i++)
    {
        mvpPassiveJoint[i]->updateJoint(true);
    }
    for(int i=0; i<mvpActiveJoint.size(); i++)
    {
        mvpActiveJoint[i]->updateJoint(true);
    }

    if(iter == MAXITER)
    {
        std::cout << "unable to solve " << MAXITER << std::endl;
        return false;
    }
    else
    {
        std::cout << "solve with " << iter+1 << std::endl;
        return true;
    }
}

void Robot::print(bool _body, bool _joint, bool _endeffector)
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

    if(_endeffector)
    {
        for(auto &ee : mvpEndEffector)
        {
            ee->print();
            std::cout << "----------------------\n";
        }
    }
}

std::vector<double> Robot::getq(bool _bActive, bool _bPassive)
{
    std::vector<double> ans;
    if(_bActive)
    {
        for(auto &joint : mvpActiveJoint)
        {
            ans.push_back(joint->getAngle());
        }
    }
    if(_bPassive)
    {
        for(auto &joint : mvpPassiveJoint)
        {
            ans.push_back(joint->getAngle());
        }
    }
    return ans;
}


