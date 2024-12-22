#ifndef HAPTIC_HPP
#define HAPTIC_HPP

#include <iostream>
#include <cstdio>
#include <cassert>
#include <time.h>
#include <fstream>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <Eigen/Dense>

#include "../lib/my_lib_for_ik/facade/main_kinematic.hpp"

class MainHaptic
{
private:

    clock_t last_time_;
    clock_t init_time_;

    hduVector3Dd previous_position_; // Предыдущее положение хаптика
    hduVector3Dd position_;          // Положение хаптика
    hduVector3Dd delta_position_;    // Смещение хаптика

    hduVector3Dd joint_angles_;      // Углы в джоинтах
    hduVector3Dd wrist_angles_;      // Углы в кулаке
    int nButtons_ = 0;
    bool btn_1_;
    bool btn_2_;

    Eigen::Vector3d initial_pos_;
    Eigen::Vector3d temp_;
    Eigen::Vector3d current_pos_;
    Eigen::Matrix<double,3,3> current_rot_;

    const double radius_ = 0.5;

    const double dh_theta_[6] = {-M_PI/2, 0, 0, 0, -M_PI/2, 0};                      // Параметры ДХ хаптика
    const double dh_alpha_[6] = {M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0};     // Параметры ДХ хаптика

    Eigen::Matrix<double,3,3> R(double theta, double alpha);                  // Матрица поворота
    Eigen::Matrix<double,3,3> FK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles);   // Прямая кинематика для хаптика (только матрица поворота)

    Kinematic* kinematic_;

    static HDCallbackCode HDCALLBACK Callback(void *data);

    bool checkPos(Eigen::Vector3d current);

public:

    MainHaptic();
    ~MainHaptic();

    int hapticInit();

};

#endif