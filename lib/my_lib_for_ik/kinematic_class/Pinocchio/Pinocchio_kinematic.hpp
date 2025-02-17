#ifndef PIN_KINEMATIC_HPP
#define PIN_KINEMATIC_HPP

#include <Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include "../base_kinematic.hpp"

#include <iostream>
#include <vector>
#include <string>

using namespace iiwa_kinematics;

class PinKinematic: public BaseKinematic
{
private: 

    // --------------------------------------------- Для инициализации

    pinocchio::Model model_;
    pinocchio::Data data_;

    Eigen::Matrix<double, N_JOINTS, 1> thetta_max_;
    Eigen::Matrix<double, N_JOINTS, 1> thetta_min_;

    // --------------------------------------------- Расчеты

    Eigen::Matrix<double, N_JOINTS, 1> thetta_previous_;
    Eigen::Matrix<double, N_JOINTS, 1> thetta_;

    // Eigen::Matrix<double,N_JOINTS,3> joint_pose_;            // Вектор положения углов

    pinocchio::SE3 endefector_previous_;                // Положение эндефектора на предыдущем шаге
    pinocchio::SE3 endefector_;                         // Положение эндефектора

    // --------------------------------------------- Вспомогательное

    const double eps_ = 1e-4;
    const int IT_MAX_ = 1000;
    const double DT_ = 1e-1;
    const double damp_ = 1e-6;

    pinocchio::Data::Matrix6x J_;
    pinocchio::Data::Matrix6 Jlog_;
    pinocchio::Data::Matrix6 JJt_;

    bool success_ = false;
    Eigen::Matrix<double, 6, 1> err_;
    Eigen::VectorXd v_;

public:
    PinKinematic(std::string urdf_name);
    ~PinKinematic();

    void setQ(const Eigen::Array<double,N_JOINTS,1> &thetta) override;
    Eigen::Array<double,N_JOINTS,1> getQ() override;

    void setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta) override;
    Eigen::Array<double,N_JOINTS,1> getNullBias() override;

    void setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation) override;
    Eigen::Matrix<double,3,3> getRotationMatrix() override;

    void setPositionVector(const Eigen::Vector3d &position) override;
    Eigen::Vector3d getPositionVector() override;

    Eigen::Matrix<double,N_JOINTS,3> getJointPose();

    Eigen::Matrix<double,6,1> getForce(const Eigen::Array<double,N_JOINTS,1> &thetta, const Eigen::Array<double,N_JOINTS,1> &torque);

    int FK() override;
    // bool FK(const Eigen::Array<double,N_JOINTS,1> &thetta) override;

    int IK() override;
    // bool IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position) override;
    
};

#endif