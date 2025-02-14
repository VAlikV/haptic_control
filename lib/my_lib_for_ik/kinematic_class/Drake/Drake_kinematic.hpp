#ifndef DRAKE_KINEMATIC_HPP
#define DRAKE_KINEMATIC_HPP

#include <Eigen/Dense>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/find_resource.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/common/drake_assert.h>
#include <drake/geometry/scene_graph.h>
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/solve.h"
#include "drake/math/rotation_matrix.h"

#include "../base_kinematic.hpp"

#include <iostream>
#include <vector>
#include <string>

using namespace iiwa_kinematics;

class DrakeKinematic: public BaseKinematic
{
private: 

    // --------------------------------------------- Для инициализации

    drake::multibody::MultibodyPlant<double> plant_;
    std::unique_ptr<drake::systems::Context<double>> context_;

    // drake::multibody::InverseKinematics* ik_;

    Eigen::Matrix<double, N_JOINTS, 1> thetta_max_;
    Eigen::Matrix<double, N_JOINTS, 1> thetta_min_;

    // --------------------------------------------- Расчеты

    Eigen::Matrix<double, N_JOINTS, 1> thetta_previous_;
    Eigen::Matrix<double, N_JOINTS, 1> thetta_;

    Eigen::Vector3d position_previous_;
    Eigen::Vector3d position_;

    Eigen::Matrix3d rotation_previous_;
    Eigen::Matrix3d rotation_;

    // --------------------------------------------- Вспомогательное

    Eigen::Vector3d p_BQ_;          // Точка Q в системе координат B
    Eigen::Vector3d lower_bound_;   // Нижняя граница позиции
    Eigen::Vector3d upper_bound_;   // Верхняя граница позиции

    const drake::math::RotationMatrix<double> identity_orientation_;    // Эталонная ориентация (мировая)

    drake::math::RotationMatrix<double> target_orientation_;            // Целевая ориентация (для перезаписи)

    drake::math::RigidTransform<double> rt_;
    

    // Eigen::Matrix<double,N_JOINTS,3> joint_pose_;            // Вектор положения углов

    

public:
    DrakeKinematic(std::string urdf_name);
    ~DrakeKinematic();

    void setQ(const Eigen::Array<double,N_JOINTS,1> &thetta) override;
    Eigen::Array<double,N_JOINTS,1> getQ() override;

    void setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta) override;
    Eigen::Array<double,N_JOINTS,1> getNullBias() override;

    void setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation) override;
    Eigen::Matrix<double,3,3> getRotationMatrix() override;

    void setPositionVector(const Eigen::Vector3d &position) override;
    Eigen::Vector3d getPositionVector() override;

    Eigen::Matrix<double,N_JOINTS,3> getJointPose();

    int FK() override;
    // bool FK(const Eigen::Array<double,N_JOINTS,1> &thetta) override;

    int IK() override;
    // bool IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position) override;
    
};

#endif