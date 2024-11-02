#ifndef _ENDEFFECTOR_H__
#define _ENDEFFECTOR_H__

#include <memory>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "robotics/body.h"


class EndEffector
{
public:
    EndEffector(std::shared_ptr<Body> _pBody, Eigen::Matrix4d _T);

    void setId(int _id){mnid = _id;}

    int getId(){return mnid;}

    void update();

    Eigen::Matrix4d getT(){return mTGlobalEE;}

    void setPose(Eigen::Matrix4d _T);

    void resetIKMode(){mpBody.lock()->resetIKMode();}

    void setIKMode(bool _mode){mpBody.lock()->setIKMode(_mode);}

    void print();

private:
    Eigen::Matrix4d mTBody2EE;

    Eigen::Matrix4d mTGlobalEE;

    std::weak_ptr<Body> mpBody;

    int mnid;

    
};
#endif // _ENDEFFECTOR_H__