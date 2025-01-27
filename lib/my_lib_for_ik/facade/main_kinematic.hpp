#ifndef MAIN_KINEMATIC_HPP
#define MAIN_KINEMATIC_HPP

#include <Eigen/Eigen>
#include "../kinematic_class/base_kinematic.hpp"
#include "../kinematic_class/KDL/KDL_kinematic.hpp"
#include "../kinematic_class/SNS/SNS_kinematic.hpp"
#include "../kinematic_class/Pinocchio/Pinocchio_kinematic.hpp"
#include "../kinematic_class/Drake/Drake_kinematic.hpp"

using namespace iiwa_kinematics;

class Kinematic
{
private:
    BaseKinematic* kinematic_solver_;

public:

    Kinematic(BaseKinematic* kinematic_solver);
    ~Kinematic();

    // ----------------------------------------------------------------------- Сетеры/Гетеры

    void setQRad(const Eigen::Array<double,N_JOINTS,1> &thetta);
    Eigen::Array<double,N_JOINTS,1> getQRad();

    void setQDeg(const Eigen::Array<double,N_JOINTS,1> &thetta);
    Eigen::Array<double,N_JOINTS,1> getQDeg();

    void setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation);
    Eigen::Matrix<double,3,3> getRotationMatrix();

    void setPositionVector(const Eigen::Array<double,3,1> &position);
    Eigen::Array<double,3,1> getPositionVector();

    Eigen::Matrix<double,N_JOINTS,3> getJointPose();

    // ----------------------------------------------------------------------- Кинематика

    int FK();
    int FK(const Eigen::Array<double,N_JOINTS,1> &thetta);

    int IK();
    int IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position);

    // ----------------------------------------------------------------------- Вспомогаетльное

    Eigen::Array<double,N_JOINTS,1> rad2deg(const Eigen::Array<double,N_JOINTS,1> &thetta);
    Eigen::Array<double,N_JOINTS,1> deg2rad(const Eigen::Array<double,N_JOINTS,1> &thetta);

};

#endif