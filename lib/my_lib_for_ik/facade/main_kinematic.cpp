#include "main_kinematic.hpp"

Kinematic::Kinematic(BaseKinematic* kinematic_solver) : kinematic_solver_(kinematic_solver)
{

}

Kinematic::~Kinematic()
{
    delete kinematic_solver_;
    kinematic_solver_ = nullptr;
}

// ----------------------------------------------------------------------- Вспомогаетльное

Eigen::Array<double,N_JOINTS,1> Kinematic::rad2deg(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    return thetta*180/M_PI;
}

Eigen::Array<double,N_JOINTS,1> Kinematic::deg2rad(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    return thetta*M_PI/180;
}

// ----------------------------------------------------------------------- Сетеры/Гетеры

void Kinematic::setQRad(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    kinematic_solver_->setQ(thetta);
}

void Kinematic::setQDeg(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    kinematic_solver_->setQ(this->deg2rad(thetta));
}

Eigen::Array<double,N_JOINTS,1> Kinematic::getQRad()
{
    return kinematic_solver_->getQ();
}

Eigen::Array<double,N_JOINTS,1> Kinematic::getQDeg()
{
    return this->rad2deg(kinematic_solver_->getQ());
}

// -----------------------

void Kinematic::setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation)
{
    kinematic_solver_->setRotationMatrix(rotation);
}

Eigen::Matrix<double,3,3> Kinematic::getRotationMatrix()
{
    return kinematic_solver_->getRotationMatrix();
}

// -----------------------

void Kinematic::setPositionVector(const Eigen::Array<double,3,1> &position)
{
    kinematic_solver_->setPositionVector(position);
}

Eigen::Array<double,3,1> Kinematic::getPositionVector()
{
    return kinematic_solver_->getPositionVector();
}

// -----------------------

Eigen::Matrix<double,N_JOINTS,3> Kinematic::getJointPose()
{
    return kinematic_solver_->getJointPose();
}

// -----------------------

Eigen::Matrix<double,6,1> Kinematic::getForce(const Eigen::Array<double,N_JOINTS,1> &thetta, const Eigen::Array<double,N_JOINTS,1> &torque)
{
    return kinematic_solver_->getForce(thetta, torque);
}

// ----------------------------------------------------------------------- Кинематика

int Kinematic::FK()
{
    return kinematic_solver_->FK();
}

int Kinematic::FK(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    this->setQRad(thetta);
    return this->FK();
}

int Kinematic::IK()
{
    return kinematic_solver_->IK();
}

int Kinematic::IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position)
{
    this->setRotationMatrix(rotation);
    this->setPositionVector(position);
    return this->IK();
}
