#ifndef _BODY_H__
#define _BODY_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <memory>
#include <iostream>
#include <vector>
#include <iostream>

class Body
{
public:
    Body(Eigen::Matrix4d _TFrame, bool _bFixed = false);

    // void insertJoint(std::shared_ptr<Joint> _pJoint, Eigen::Matrix4d _TJointCenter);

    void setId(int _id);

    int getId(){return mnid;}

    void updatePose(Eigen::Matrix4d _T){mTFrame = _T;}

    Eigen::Matrix4d getT(){return mTFrame;}

    bool isFixed(){return mbFixed;}

    void print();

private:
    Eigen::Matrix4d mTFrame;

    // std::vector<std::weak_ptr<Joint>> mvpJoint;

    bool mbFixed{false};

    int mnid{-1};
};

#endif // _BODY_H__