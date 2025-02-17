#include "Drake_kinematic.hpp"

DrakeKinematic::DrakeKinematic(std::string urdf_name):
plant_(0.0), 
identity_orientation_(drake::math::RotationMatrix<double>::Identity())
{

    drake::multibody::Parser parser(&plant_);
    parser.AddModels(urdf_name);  // Укажите путь к модели робота
    plant_.Finalize();

    context_ = plant_.CreateDefaultContext();

    for (int i = 0; i < N_JOINTS; ++i)
    {
        thetta_max_[i] = LIMITS_MAX[i]*M_PI/180;
        thetta_min_[i] = LIMITS_MIN[i]*M_PI/180;

        thetta_previous_[i] = INIT[i];
        thetta_[i] = INIT[i];
        // thetta_previous_(i) = 0.785;
        // thetta_(i) = 0.785;
        // thetta_previous_(i) = 1.57;
        // thetta_ (i) = 1.57;
        // thetta_previous_(i) = 0;
        // thetta_(i) = 0;
    }

    this->FK();

    // ik_ = new drake::multibody::InverseKinematics(plant_);

}

DrakeKinematic::~DrakeKinematic()
{
    // delete ik_;
    // ik_ = nullptr;
}

// ----------------------------------------------------------------------- Сетеры/Гетеры

void DrakeKinematic::setQ(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    thetta_ = thetta.matrix();
}

Eigen::Array<double,N_JOINTS,1> DrakeKinematic::getQ()
{
    return thetta_.array();
}

// -----------------------

void DrakeKinematic::setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta)
{

}

Eigen::Array<double,N_JOINTS,1> DrakeKinematic::getNullBias()
{
    Eigen::Array<double,N_JOINTS,1> nulb;
    return nulb;
}

// -----------------------

void DrakeKinematic::setPositionVector(const Eigen::Vector3d &position)
{
    position_ = position;
}

Eigen::Vector3d DrakeKinematic::getPositionVector()
{
    return position_;
}

// -----------------------

void DrakeKinematic::setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation)
{
    rotation_ = rotation;
}

Eigen::Matrix<double,3,3> DrakeKinematic::getRotationMatrix()
{
    return rotation_;
}

// -----------------------

Eigen::Matrix<double,N_JOINTS,3> DrakeKinematic::getJointPose()
{
    Eigen::Matrix<double,N_JOINTS,3> joint_pose;

    // for (int i = 0; i < N_JOINTS; ++i)
    // {

    //     rt_ = plant_.CalcRelativeTransform(*context_, plant_.GetFrameByName("iiwa_link_0"), plant_.GetFrameByName("iiwa_link_" + std::to_string(i)));

    //     joint_pose.row(i) = rt_.translation();
    // }
    
    return joint_pose;
}

// -----------------------

Eigen::Matrix<double,6,1> DrakeKinematic::getForce(const Eigen::Array<double,N_JOINTS,1> &thetta, const Eigen::Array<double,N_JOINTS,1> &torque)
{
    // Eigen::Matrix<double,3,1> force;

    const drake::multibody::Body<double>& end_effector = plant_.GetBodyByName("iiwa_link_ee");

    // Устанавливаем положение в контексте
    plant_.SetPositions(context_.get(), thetta.matrix());

    Eigen::Vector3d p_BoBp_B(0, 0, 0);
    Eigen::MatrixX<double> J(6, plant_.num_velocities());

    plant_.CalcJacobianSpatialVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kV, end_effector.body_frame(),
        p_BoBp_B,  // Позиция Bp относительно Bo в координатах B
        plant_.world_frame(),  // Скорости относительно мира
        plant_.world_frame(),  // Выражаем Якобиан в мировой системе
        &J);

    // Вычисляем геометрический Якобиан J (6×n)
    // Eigen::MatrixX<double> J = plant_.CalcJacobianSpatialVelocity(
    //     *context_, drake::multibody::JacobianWrtVariable::kV, end_effector.body_frame(),
    //     end_effector.body_frame().CalcPoseInWorld(*context_).translation(),
    //     plant_.world_frame(), plant_.world_frame());

    // std::cout << "Якобиан J:\n" << J << std::endl;

    // Вычисляем силу на эндеффекторе: f = J^(-T) * τ

    Eigen::Vector<double,7> tau = torque.matrix();

    std::cout << "SIZE=========================== " << tau.size() << std::endl;


    Eigen::VectorXd force = J.transpose().fullPivHouseholderQr().solve(tau);

    std::cout << "SIZE=========================== " << force.size() << std::endl;
    
    return force;
}

// ----------------------------------------------------------------------- Кинематика

int DrakeKinematic::FK()
{
    thetta_previous_ = thetta_;
    plant_.SetPositions(context_.get(), thetta_);

    auto X_WE = plant_.CalcRelativeTransform(*context_, plant_.GetFrameByName("iiwa_link_0"), plant_.GetFrameByName("iiwa_link_ee"));

    position_ = X_WE.translation().transpose();
    rotation_ = X_WE.rotation().matrix();

    return 1;
}

// bool KDLKinematic::FK(const Eigen::Array<double,N_JOINTS,1> &thetta)
// {
//     this->setQ(thetta);
//     return this->FK();
// }

int DrakeKinematic::IK()
{
    position_previous_ = position_;
    rotation_previous_ = rotation_;
    thetta_previous_ = thetta_;

    drake::multibody::InverseKinematics* ik_ = new drake::multibody::InverseKinematics(plant_);

    target_orientation_ = drake::math::RotationMatrix<double>(rotation_.transpose());
    const drake::multibody::Frame<double>& end_effector_frame = plant_.GetFrameByName("iiwa_link_ee");

    p_BQ_ = Eigen::Vector3d::Zero();    // Точка Q в системе координат B
    lower_bound_ = position_;           // Нижняя граница позиции
    upper_bound_ = position_;           // Верхняя граница позиции

    ik_->AddPositionConstraint(
        end_effector_frame, p_BQ_, plant_.world_frame(),
        lower_bound_, upper_bound_);

    ik_->AddOrientationConstraint(
        plant_.world_frame(), identity_orientation_,  // Фиксированная ориентация для мира
        end_effector_frame, target_orientation_,     // Целевая ориентация для эндэффектора
        0.0);                                       // Угол отклонения (радианы)  

    const drake::solvers::MathematicalProgram* prog = &(ik_->prog());

    Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(plant_.num_positions());
    const drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(*prog, /* initial_guess */ thetta_);

    if (result.is_success()) {
        thetta_ = result.GetSolution(ik_->q());

        delete ik_;
        ik_ = nullptr;

        return 1;

    } else {
        thetta_ = result.GetSolution(ik_->q());

        delete ik_;
        ik_ = nullptr;
        
        return -1;
    }

    return 0;
}

// bool KDLKinematic::IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position)
// {
//     this->setRotationMatrix(rotation);
//     this->setPositionVector(position);
//     return this->IK();
// }

