#include "Pinocchio_kinematic.hpp"

PinKinematic::PinKinematic(std::string urdf_name)
{
    pinocchio::urdf::buildModel(urdf_name, model_);
    data_ = pinocchio::Data(model_);
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

    J_ = pinocchio::Data::Matrix6x(6, model_.nv);
    J_.setZero();

    v_ = Eigen::VectorXd(model_.nv);
}

PinKinematic::~PinKinematic()
{
    
}

// ----------------------------------------------------------------------- Сетеры/Гетеры

void PinKinematic::setQ(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    thetta_ = thetta.matrix();
}

Eigen::Array<double,N_JOINTS,1> PinKinematic::getQ()
{
    return thetta_.array();
}

// -----------------------

void PinKinematic::setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta)
{

}

Eigen::Array<double,N_JOINTS,1> PinKinematic::getNullBias()
{
    Eigen::Array<double,N_JOINTS,1> nulb;
    return nulb;
}

// -----------------------

void PinKinematic::setPositionVector(const Eigen::Vector3d &position)
{
    endefector_.translation() = position;
}

Eigen::Vector3d PinKinematic::getPositionVector()
{
    return endefector_.translation();
}

// -----------------------

void PinKinematic::setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation)
{
    endefector_.rotation() = rotation;
}

Eigen::Matrix<double,3,3> PinKinematic::getRotationMatrix()
{
    return endefector_.rotation();
}

// -----------------------

Eigen::Matrix<double,N_JOINTS,3> PinKinematic::getJointPose()
{
    Eigen::Matrix<double,N_JOINTS,3> joint_pose;

    for (int i = 0; i < N_JOINTS; ++i)
    {
        joint_pose.row(i) = data_.oMi[i+1].translation();
    }
    
    return joint_pose;
}

// -----------------------

Eigen::Matrix<double,6,1> PinKinematic::getForce(const Eigen::Array<double,N_JOINTS,1> &thetta, const Eigen::Array<double,N_JOINTS,1> &torque)
{
    Eigen::Matrix<double,6,1> force;
    
    return force;
}

// ----------------------------------------------------------------------- Кинематика

int PinKinematic::FK()
{
    thetta_previous_ = thetta_;
    pinocchio::forwardKinematics(model_, data_, thetta_);
    endefector_ = data_.oMi[7];
    return 1;
}

// bool KDLKinematic::FK(const Eigen::Array<double,N_JOINTS,1> &thetta)
// {
//     this->setQ(thetta);
//     return this->FK();
// }

int PinKinematic::IK()
{
    endefector_previous_ = endefector_;
    thetta_previous_ = thetta_;

    pinocchio::SE3 oMdes(endefector_.rotation(), endefector_.translation());

    for (int i = 0;; i++)
    {
        pinocchio::forwardKinematics(model_, data_, thetta_);
        const pinocchio::SE3 iMd = data_.oMi[N_JOINTS].actInv(oMdes);
        err_ = pinocchio::log6(iMd).toVector(); // in joint frame

        if (err_.norm() < eps_)
        {
            success_ = true;
            break;
        }
        if (i >= IT_MAX_)
        {
            success_ = false;
            break;
        }

        pinocchio::computeJointJacobian(model_, data_, thetta_, N_JOINTS, J_); // J in joint frame
        pinocchio::Jlog6(iMd.inverse(), Jlog_);

        J_ = -Jlog_ * J_;

        JJt_.noalias() = J_ * J_.transpose();
        JJt_.diagonal().array() += damp_;
        v_.noalias() = -J_.transpose() * JJt_.ldlt().solve(err_);
        thetta_ = pinocchio::integrate(model_, thetta_, v_ * DT_);
        // if (!(i % 10))
        // std::cout << i << ": error = " << err.transpose() << std::endl;
    }

    if (success_)
    {
        // std::cout << "Convergence achieved!" << std::endl;
        return 1;
    }
    else
    {
        // std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
        return -1;
    }

    // return 0;
}

// bool KDLKinematic::IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position)
// {
//     this->setRotationMatrix(rotation);
//     this->setPositionVector(position);
//     return this->IK();
// }

