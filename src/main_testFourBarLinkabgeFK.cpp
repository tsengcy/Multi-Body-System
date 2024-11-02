#include "robotics/robot.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

int main()
{
       std::shared_ptr<Robot> pRobot(new Robot());

       Eigen::Matrix4d mat;
       // Frame 0
       mat << 1, 0, 0, 1,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
       pRobot->insertBody(mat, true);
       
       // Frame 1
       mat << 0, -1, 0, 0,
              1,  0, 0, 1,
              0,  0, 1, 0,
              0,  0, 0, 1;
       pRobot->insertBody(mat);

       // Frame 2
       mat << 1, 0, 0, 1,
              0, 1, 0, 2,
              0, 0, 1, 0,
              0, 0, 0, 1;
       pRobot->insertBody(mat);

       // Frame 4
       mat << 0, -1, 0, 2,
              1,  0, 0, 1,
              0,  0, 1, 0,
              0,  0, 0, 1;
       pRobot->insertBody(mat);

       // joint 0
       mat << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
       pRobot->insertJoint(mat, 0, 1, ACTIVE);

       // joint 0
       mat << 1, 0, 0, 0,
              0, 1, 0, 2,
              0, 0, 1, 0,
              0, 0, 0, 1;
       pRobot->insertJoint(mat, 1, 2, PASSIVE);

       // joint 0
       mat << 1, 0, 0, 2,
              0, 1, 0, 2,
              0, 0, 1, 0,
              0, 0, 0, 1;
       pRobot->insertJoint(mat, 2, 3, PASSIVE);

       // joint 0
       mat << 1, 0, 0, 2,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
       pRobot->insertJoint(mat, 3, 0, PASSIVE);

       std::vector<double> angles{-45.0 / 180.0 * M_PI};
       pRobot->setActiveJointAngle(angles);
       pRobot->ForwardKinematics();


       pRobot->print(true, true);
}