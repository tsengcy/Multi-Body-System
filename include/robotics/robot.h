#ifndef _ROBOT_H__
#define _ROBOT_H__

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "robotics/body.h"
#include "robotics/joint.h"
#include "robotics/endeffector.h"

class Robot
{
public:
    Robot();

    void insertBody(Eigen::Matrix4d _T, bool _bFixed = false);

    void insertJoint(Eigen::Matrix4d _T, int _nParentId, int _nChildId, JointType _type);

    void insertEndEffector(Eigen::Matrix4d _T, int _nParentId);

    void setActiveJointAngle(std::vector<double> _angle);

    bool ForwardKinematics();

    bool InverseKinematics(std::vector<int> _vEEId, std::vector<Eigen::Matrix4d> _vEEFrame);

    void print(bool _body = true, bool _joint = true, bool _endeffector = true);

    std::vector<double> getq(bool _bActive=true, bool _bPassive=true);

    Eigen::Matrix4d getEEPose(int _id){return mvpEndEffector[_id]->getT();}

private:
    std::vector<std::shared_ptr<Body>> mvpBody;

    std::vector<std::shared_ptr<Joint>> mvpActiveJoint;

    std::vector<std::shared_ptr<Joint>> mvpPassiveJoint;

    std::vector<std::shared_ptr<EndEffector>> mvpEndEffector;

    int mnMoveBody{0};

    int mnFixedBody{0};
};



#endif // _ROBOT_H__