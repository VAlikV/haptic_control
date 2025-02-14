#include "SNS_kinematic.hpp"

SNSKinematic::SNSKinematic(sns_ik::VelocitySolveType type)
{
    chain_.addSegment(KDL::Segment("Link_1",
                                KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Vector(0.0,0.0,LINKS[0])),
                                KDL::RigidBodyInertia(LINKS_MASS[0],
                                                    KDL::Vector::Zero(),
                                                    KDL::RotationalInertia(LINKS_INERTIA[0][0],LINKS_INERTIA[0][1],LINKS_INERTIA[0][2])
                                                    )
                                )
                    );

    chain_.addSegment(KDL::Segment("Link_2",
                                KDL::Joint(KDL::Joint::RotX),
                                KDL::Frame(KDL::Vector(0.0,0.0,LINKS[1])),
                                KDL::RigidBodyInertia(LINKS_MASS[1],
                                                    KDL::Vector::Zero(),
                                                    KDL::RotationalInertia(LINKS_INERTIA[1][0],LINKS_INERTIA[1][1],LINKS_INERTIA[1][2])
                                                    )
                                )
                    );

    chain_.addSegment(KDL::Segment("Link_3",
                                KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Vector(0.0,0.0,LINKS[2])),
                                KDL::RigidBodyInertia(LINKS_MASS[2],
                                                    KDL::Vector::Zero(),
                                                    KDL::RotationalInertia(LINKS_INERTIA[2][0],LINKS_INERTIA[2][1],LINKS_INERTIA[2][2])
                                                    )
                                )
                    );

    chain_.addSegment(KDL::Segment("Link_4",
                                KDL::Joint(KDL::Joint::RotX),
                                KDL::Frame(KDL::Vector(0.0,0.0,LINKS[3])),
                                KDL::RigidBodyInertia(LINKS_MASS[3],
                                                    KDL::Vector::Zero(),
                                                    KDL::RotationalInertia(LINKS_INERTIA[3][0],LINKS_INERTIA[3][1],LINKS_INERTIA[3][2])
                                                    )
                                )
                    );

    chain_.addSegment(KDL::Segment("Link_5",
                                KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Vector(0.0,0.0,LINKS[4])),
                                KDL::RigidBodyInertia(LINKS_MASS[4],
                                                    KDL::Vector::Zero(),
                                                    KDL::RotationalInertia(LINKS_INERTIA[4][0],LINKS_INERTIA[4][1],LINKS_INERTIA[4][2])
                                                    )
                                )
                    );

    chain_.addSegment(KDL::Segment("Link_6",
                                KDL::Joint(KDL::Joint::RotX),
                                KDL::Frame(KDL::Vector(0.0,0.0,LINKS[5])),
                                KDL::RigidBodyInertia(LINKS_MASS[5],
                                                    KDL::Vector::Zero(),
                                                    KDL::RotationalInertia(LINKS_INERTIA[5][0],LINKS_INERTIA[5][1],LINKS_INERTIA[5][2])
                                                    )
                                )
                    );

    chain_.addSegment(KDL::Segment("Link_7",
                                KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Vector(0.0,0.0,LINKS[6])),
                                KDL::RigidBodyInertia(LINKS_MASS[6],
                                                    KDL::Vector::Zero(),
                                                    KDL::RotationalInertia(LINKS_INERTIA[6][0],LINKS_INERTIA[6][1],LINKS_INERTIA[6][2])
                                                    )
                                )
                    );

    std::vector<std::string> names = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_7"}; 
    std::vector<std::string> biasNames_ = {"Bias_joint_1", "Bias_joint_2", "Bias_joint_3", "Bias_joint_4", "Bias_joint_5", "Bias_joint_6", "Bias_joint_7"}; 
    thetta_max_ = KDL::JntArray(N_JOINTS);
    thetta_min_ = KDL::JntArray(N_JOINTS);
    velocity_max_ = KDL::JntArray(N_JOINTS);
    acceleration_max_ = KDL::JntArray(N_JOINTS);

    thetta_previous_ = KDL::JntArray(N_JOINTS);
    thetta_ = KDL::JntArray(N_JOINTS);

    for (int i = 0; i < N_JOINTS; ++i)
    {
        thetta_max_(i) = LIMITS_MAX[i]*M_PI/180;
        thetta_min_(i) = LIMITS_MIN[i]*M_PI/180;

        velocity_max_(i) = LIMITS_VELOCITY[i];
        acceleration_max_(i) = LIMITS_ACCELERATION[i];


        thetta_previous_(i) = 0.1785;
        thetta_(i) = 0.1785;
        // thetta_previous_(i) = 0.785;
        // thetta_(i) = 0.785;
        // thetta_previous_(i) = 1.57;
        // thetta_ (i) = 1.57;
        // thetta_previous_(i) = 0;
        // thetta_(i) = 0;
    }

    fksolver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    // vsolver_ = new KDL::ChainIkSolverVel_pinv_givens(chain_);
    // vsolver_ = new KDL::ChainIkSolverVel_pinv(chain_);
    // vsolver_ = new KDL::ChainIkSolverVel_pinv_nso(chain_);

    // iksolver_ = new KDL::ChainIkSolverPos_NR_JL(chain_, thetta_min_, thetta_max_, *fksolver_, *vsolver_);
    
    joint_pose_ = std::vector<KDL::Frame>(N_JOINTS);

    fksolver_->JntToCart(thetta_, joint_pose_);
    endefector_ = joint_pose_[6];
    
    sns_ = new sns_ik::SNS_IK(chain_, thetta_min_, thetta_max_, velocity_max_, acceleration_max_, names);
}

SNSKinematic::~SNSKinematic()
{
    delete fksolver_;
    fksolver_ = nullptr;
}

// ----------------------------------------------------------------------- Сетеры/Гетеры

void SNSKinematic::setQ(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    thetta_.data = thetta;
}

Eigen::Array<double,N_JOINTS,1> SNSKinematic::getQ()
{
    return thetta_.data;
}

// -----------------------

void SNSKinematic::setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta)
{

}

Eigen::Array<double,N_JOINTS,1> SNSKinematic::getNullBias()
{
    Eigen::Array<double,N_JOINTS,1> nulb;
    return nulb;
}

// -----------------------

void SNSKinematic::setPositionVector(const Eigen::Vector3d &position)
{
    endefector_.p.x(position(0));
    endefector_.p.y(position(1));
    endefector_.p.z(position(2));
}

Eigen::Vector3d SNSKinematic::getPositionVector()
{
    Eigen::Vector3d pos;
    pos << endefector_.p.x(), endefector_.p.y(), endefector_.p.z();
    return pos;
}

// -----------------------

void SNSKinematic::setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation)
{
    KDL::Rotation rot = KDL::Rotation(rotation(0,0), rotation(0,1), rotation(0,2), 
                    rotation(1,0), rotation(1,1), rotation(1,2), 
                    rotation(2,0), rotation(2,1), rotation(2,2));
    endefector_.M = rot;
}

Eigen::Matrix<double,3,3> SNSKinematic::getRotationMatrix()
{
    Eigen::Matrix<double,3,3> rot;
    rot << endefector_.M.data[0], endefector_.M.data[1], endefector_.M.data[2],
            endefector_.M.data[3], endefector_.M.data[4], endefector_.M.data[5],
            endefector_.M.data[6], endefector_.M.data[7], endefector_.M.data[8];
    return rot;
}

// -----------------------

Eigen::Matrix<double,N_JOINTS,3> SNSKinematic::getJointPose()
{
    Eigen::Matrix<double,N_JOINTS,3> joint_pose;
    
    return joint_pose;
}

// ----------------------------------------------------------------------- Кинематика

int SNSKinematic::FK()
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

int SNSKinematic::IK()
{
    endefector_previous_ = endefector_;
    thetta_previous_ = thetta_;
    return sns_->CartToJnt(thetta_previous_,endefector_,thetta_); // kinematics_status;
}

// bool KDLKinematic::IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position)
// {
//     this->setRotationMatrix(rotation);
//     this->setPositionVector(position);
//     return this->IK();
// }

