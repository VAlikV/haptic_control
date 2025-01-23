#include "KDL_kinematic.hpp"

KDLKinematic::KDLKinematic()
{
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,LINKS[0]))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,LINKS[2]))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,LINKS[4]))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,LINKS[6]))));

    thetta_max_ = KDL::JntArray(N_JOINTS);
    thetta_min_ = KDL::JntArray(N_JOINTS);
    thetta_previous_ = KDL::JntArray(N_JOINTS);
    thetta_ = KDL::JntArray(N_JOINTS);

    for (int i = 0; i < N_JOINTS; ++i)
    {
        thetta_max_(i) = LIMITS_MAX[i]*M_PI/180;
        thetta_min_(i) = LIMITS_MIN[i]*M_PI/180;

        thetta_previous_(i) = INIT[i];
        thetta_(i) = INIT[i];
        // thetta_previous_(i) = 0.785;
        // thetta_(i) = 0.785;
        // thetta_previous_(i) = 1.57;
        // thetta_ (i) = 1.57;
        // thetta_previous_(i) = 0;
        // thetta_(i) = 0;
    }

    fksolver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    // vsolver_ = new KDL::ChainIkSolverVel_pinv_givens(chain_);
    vsolver_ = new KDL::ChainIkSolverVel_pinv(chain_);
    // vsolver_ = new KDL::ChainIkSolverVel_pinv_nso(chain_);

    iksolver_ = new KDL::ChainIkSolverPos_NR_JL(chain_, thetta_min_, thetta_max_, *fksolver_, *vsolver_);
    
    joint_pose_ = std::vector<KDL::Frame>(N_JOINTS);

    fksolver_->JntToCart(thetta_, joint_pose_);
    endefector_ = joint_pose_[6];
}

KDLKinematic::~KDLKinematic()
{
    delete fksolver_;
    fksolver_ = nullptr;

    delete vsolver_;
    vsolver_ = nullptr;

    delete iksolver_;
    iksolver_ = nullptr;
}

// ----------------------------------------------------------------------- Сетеры/Гетеры

void KDLKinematic::setQ(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    thetta_.data = thetta;
}

Eigen::Array<double,N_JOINTS,1> KDLKinematic::getQ()
{
    return thetta_.data;
}

// -----------------------

void KDLKinematic::setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta)
{

}

Eigen::Array<double,N_JOINTS,1> KDLKinematic::getNullBias()
{
    Eigen::Array<double,N_JOINTS,1> nulb;
    return nulb;
}

// -----------------------

void KDLKinematic::setPositionVector(const Eigen::Array<double,3,1> &position)
{
    endefector_.p.x(position(0));
    endefector_.p.y(position(1));
    endefector_.p.z(position(2));
}

Eigen::Array<double,3,1> KDLKinematic::getPositionVector()
{
    Eigen::Array<double,3,1> pos;
    pos << endefector_.p.x(), endefector_.p.y(), endefector_.p.z();
    return pos;
}

// -----------------------

void KDLKinematic::setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation)
{
    KDL::Rotation rot = KDL::Rotation(rotation(0,0), rotation(0,1), rotation(0,2), 
                    rotation(1,0), rotation(1,1), rotation(1,2), 
                    rotation(2,0), rotation(2,1), rotation(2,2));
    endefector_.M = rot;
}

Eigen::Matrix<double,3,3> KDLKinematic::getRotationMatrix()
{
    Eigen::Matrix<double,3,3> rot;
    rot << endefector_.M.data[0], endefector_.M.data[1], endefector_.M.data[2],
            endefector_.M.data[3], endefector_.M.data[4], endefector_.M.data[5],
            endefector_.M.data[6], endefector_.M.data[7], endefector_.M.data[8];
    return rot;
}

// -----------------------

Eigen::Matrix<double,N_JOINTS,3> KDLKinematic::getJointPose()
{
    Eigen::Matrix<double,N_JOINTS,3> joint_pose;
    for (int i = 0; i < N_JOINTS; ++i)
    {
        joint_pose(i,0) = joint_pose_[i].p.x();
        joint_pose(i,1) = joint_pose_[i].p.y();
        joint_pose(i,2) = joint_pose_[i].p.z();
    }
    return joint_pose;
}

// ----------------------------------------------------------------------- Кинематика

int KDLKinematic::FK()
{
    thetta_previous_ = thetta_;
    int kinematics_status = fksolver_->JntToCart(thetta_, joint_pose_);
    endefector_ = joint_pose_[6];
    return kinematics_status;
}

// bool KDLKinematic::FK(const Eigen::Array<double,N_JOINTS,1> &thetta)
// {
//     this->setQ(thetta);
//     return this->FK();
// }

int KDLKinematic::IK()
{
    endefector_previous_ = endefector_;
    thetta_previous_ = thetta_;
    int kinematics_status = iksolver_->CartToJnt(thetta_previous_, endefector_, thetta_);
    return kinematics_status;
}

// bool KDLKinematic::IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position)
// {
//     this->setRotationMatrix(rotation);
//     this->setPositionVector(position);
//     return this->IK();
// }

