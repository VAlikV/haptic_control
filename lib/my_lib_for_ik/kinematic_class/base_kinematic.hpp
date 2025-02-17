#ifndef BASE_KINEMATIC_HPP
#define BASE_KINEMATIC_HPP

#include <fstream>
#include <iostream>
#include <Eigen/Dense>

namespace iiwa_kinematics
{
    const int N_JOINTS = 7;

    const double LIMITS_MAX[N_JOINTS] = {168, 118, 168, 118, 168, 118, 173};
    const double LIMITS_MIN[N_JOINTS] = {-168, -118, -168, -118, -168, -118, -173};

    const double LIMITS_VELOCITY[N_JOINTS] = {10, 10, 10, 10, 10, 10, 10};
    const double LIMITS_ACCELERATION[N_JOINTS] = {100, 100, 100, 100, 100, 100, 100};

    const double INIT[N_JOINTS] = {-0.19652407, 0.15269886, 0.21432643, 1.96000475, 2.93215314/2, -0.15599753, 1.40481551};

    const double LINKS[N_JOINTS] = {0.34, 0, 0.4, 0, 0.4, 0, 0.126};

    const double LINKS_MASS[N_JOINTS] = {/*5,*/ 4, 4, 3, 2.7, 1.7, 1.8, 0.3};
    const double LINKS_INERTIA[N_JOINTS][3] = {/*{0.05, 0.06, 0.03},*/
                                                {0.1, 0.09, 0.02},
                                                {0.05, 0.018, 0.044},
                                                {0.08, 0.075, 0.01},
                                                {0.03, 0.01, 0.029},
                                                {0.02, 0.018, 0.005},
                                                {0.005, 0.0036, 0.0047},
                                                {0.001, 0.001, 0.001}};
 
    const int MAX_ITER = 100;
    const double EPS = 1e-2;

    class BaseKinematic
    {
    public:

        // ----------------------------------------------------------------------- Сетеры/Гетеры

        virtual void setQ(const Eigen::Array<double,N_JOINTS,1> &thetta) = 0;
        virtual Eigen::Array<double,N_JOINTS,1> getQ() = 0;

        virtual void setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta) = 0;
        virtual Eigen::Array<double,N_JOINTS,1> getNullBias() = 0;

        virtual void setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation) = 0;
        virtual Eigen::Matrix<double,3,3> getRotationMatrix() = 0;

        virtual void setPositionVector(const Eigen::Vector3d &position) = 0;
        virtual Eigen::Vector3d getPositionVector() = 0;

        virtual Eigen::Matrix<double,N_JOINTS,3> getJointPose() = 0;

        virtual Eigen::Matrix<double,6,1> getForce(const Eigen::Array<double,N_JOINTS,1> &thetta, const Eigen::Array<double,N_JOINTS,1> &torque) = 0;       

        // ----------------------------------------------------------------------- Кинематика

        virtual int FK() = 0;
        // virtual bool FK(const Eigen::Array<double,N_JOINTS,1> &thetta) = 0;

        virtual int IK() = 0;
        // virtual bool IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position) = 0;
    };

    void saveMatrix(std::string name, Eigen::Matrix<double,N_JOINTS,3> mat);
}

#endif