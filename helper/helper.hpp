#ifndef HELPER
#define HELPER

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <Eigen/Dense>

namespace kinematics_helper
{
    const double dh_theta[6] = {-M_PI/2, 0, M_PI/2, 0, 0, 0};                // Параметры ДХ хаптика
    const double dh_alpha[6] = {-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0};     // Параметры ДХ хаптика

    Eigen::Matrix<double,3,3> R(double theta, double alpha);

    Eigen::Matrix<double,3,3> FK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles);

    bool checkPos(const Eigen::Vector3d& current, const Eigen::Vector3d& initial, const double radius);
}




#endif