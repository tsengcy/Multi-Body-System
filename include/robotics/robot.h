#ifndef _ROBOT_H__
#define _ROBOT_H__

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "robotics/body.h"
#include "robotics/joint.h"

class Robot
{
public:
    Robot();

    void insertBody(Eigen::Matrix4d _T, bool _bFixed = false);

    void insertJoint(Eigen::Matrix4d _T, int _nParentId, int _nChildId, JointType _type);

    void setActiveJointAngle(std::vector<double> _angle);

    void kinematics();

    void print(bool _body = true, bool _joint = true);

private:
    std::vector<std::shared_ptr<Body>> mvpBody;

    std::vector<std::shared_ptr<Joint>> mvpActiveJoint;

    std::vector<std::shared_ptr<Joint>> mvpPassiveJoint;
};



#endif // _ROBOT_H__