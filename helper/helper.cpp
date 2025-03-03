#include "helper.hpp"

using namespace kinematics_helper;

Eigen::Matrix<double,3,3> kinematics_helper::R(double theta, double alpha)         // Rotation matrix
{
    Eigen::Matrix<double,3,3> R;
    R << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                  0,             sin(alpha),             cos(alpha);
    return R;
}

Eigen::Matrix<double,3,3> kinematics_helper::FK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles)      // Forvard kinematics
{
    Eigen::Matrix<double,3,3> rotation = Eigen::Matrix<double,3,3>::Identity(3,3);

    for (int8_t i = 0; i < 3; ++i)
    {
        rotation = rotation * R(dh_theta[i]+joint_angles[i]*dh_m[i], dh_alpha[i]);
    }
    for (int8_t i = 3; i < 6; ++i)
    {
        rotation = rotation * R(dh_theta[i]+wrist_angles[i-3]*dh_m[i], dh_alpha[i]);
    }
    return rotation;
}

bool kinematics_helper::checkPos(const Eigen::Vector3d& current, const Eigen::Vector3d& initial, const double radius)
{
    if ((current - initial).norm() <= radius)
    {
        return true;
    }
    else
    {
        return false;
    }
}