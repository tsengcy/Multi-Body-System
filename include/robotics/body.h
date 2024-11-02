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

    void setId(int _id, int _moveid = -1);

    int getId(){return mnid;}

    int getMoveId(){return mnMoveId;}

    void updatePose(Eigen::Matrix4d _T){mTFrame = _T;}

    Eigen::Matrix4d getT(){return mTFrame;}

    bool isFixed(){return mbFixed;}

    void print();

    void resetIKMode(){mbIk = false;}

    void setIKMode(bool _mode){mbIk = _mode;}

    bool getIKMode(){return mbIk || mbFixed;}

private:
    Eigen::Matrix4d mTFrame;

    // std::vector<std::weak_ptr<Joint>> mvpJoint;

    bool mbFixed{false};

    int mnid{-1};

    int mnMoveId{-1};

    bool mbIk{false};
};

#endif // _BODY_H__