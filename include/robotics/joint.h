#ifndef _JOINT_H__
#define _JOINT_H__

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>


#include "robotics/body.h"

enum JointType
{
    ACTIVE = 0,
    PASSIVE = 1
};


class Joint
{
public:
    Joint(std::shared_ptr<Body> _Parent, std::shared_ptr<Body> _Child, Eigen::Matrix4d _TFrame, JointType _type);

    void setId(int _id){mnid = _id;}

    void setAngle(double _angle){mAngle = _angle;}

    std::shared_ptr<Body> getParent(){return mpParent.lock();}

    Eigen::Matrix4d getTParent2Joint(){return mTParent2Joint;}

    int getParentId(){return mnParentId;}

    std::shared_ptr<Body> getChild(){return mpChild.lock();}

    Eigen::Matrix4d getTChild2Joint(){return mTChild2Joint;}

    int getChildId(){return mnChildId;}

    Eigen::Matrix4d getTransformation();

    void print();

private:
    std::weak_ptr<Body> mpParent;

    Eigen::Matrix4d mTParent2Joint;

    int mnParentId;

    std::weak_ptr<Body> mpChild;

    Eigen::Matrix4d mTChild2Joint;

    int mnChildId;

    int mnid;

    double mAngle{0};

    JointType mJointType;

    

    
};

#endif //_JOINT_H__