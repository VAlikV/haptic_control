#ifndef KDL_KINEMATIC_HPP
#define KDL_KINEMATIC_HPP

#include "../base_kinematic.hpp"
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>

using namespace iiwa_kinematics;
// using namespace KDL;

class KDLKinematic: public BaseKinematic
{
private: 

    // --------------------------------------------- Для инициализации

    KDL::Chain chain_;                              // Цепочка манипулятора

    KDL::ChainFkSolverPos_recursive* fksolver_;      // Решатель прямой задачи кинематики

    KDL::ChainIkSolverVel_pinv* vsolver_;            // Решатель дифференциальной задачи кинематики
    // KDL::ChainIkSolverVel_pinv_givens* vsolver_;            // Решатель дифференциальной задачи кинематики
    // KDL::ChainIkSolverVel_pinv_nso* vsolver_;
    
    KDL::ChainIkSolverPos_NR_JL* iksolver_;          // Решатель обратной задачи кинематики
    // KDL::ChainIkSolverPos_NR_JL* iksolver_;

    KDL::JntArray thetta_max_;                      // Максимальные углы в джоинтах
    KDL::JntArray thetta_min_;                      // Минимальные углы в джоинтах

    // --------------------------------------------- Расчеты

    KDL::JntArray thetta_previous_;                 // Значения углов на предыдущем шаге
    KDL::JntArray thetta_;                          // Текущие значения углов

    std::vector<KDL::Frame> joint_pose_;            // Вектор положения углов

    KDL::Frame endefector_previous_;                // Положение эндефектора на предыдущем шаге
    KDL::Frame endefector_;                         // Положение эндефектора

public:
    KDLKinematic();
    ~KDLKinematic();

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