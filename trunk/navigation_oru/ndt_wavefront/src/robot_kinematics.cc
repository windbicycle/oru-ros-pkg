#include <robot_kinematics.h>

using namespace lslgeneric;
using namespace std;


RobotKinematics::RobotKinematics() {
    radius2d=radius3d = 0;
}

RobotKinematics::RobotKinematics(Eigen::Vector3d _robotSize, Eigen::Vector3d _inclinationConstraints) {
    robotSize = _robotSize;
    inclinationConstraints = _inclinationConstraints;
    computeRad();
}

double RobotKinematics::robotRadius2d() const {
    return radius2d;
}

double RobotKinematics::robotRadius3d() const {
    return radius3d;
}

Eigen::Vector3d RobotKinematics::getRobotSize() const {
    return robotSize;
}

Eigen::Vector3d RobotKinematics::getInclConstraint() const {
    return inclinationConstraints;
}

void RobotKinematics::computeRad() {
    radius2d = sqrt(pow(robotSize(0),2)+pow(robotSize(1),2))/2;
    radius3d = robotSize.norm()/2; //x/2 and y/2, not z!!
}
