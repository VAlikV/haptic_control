#ifndef HAPTIC_HANDLING
#define HAPTIC_HANDLING

#include <iostream>
#include <cstdio>
#include <cassert>
#include <time.h>
#include <fstream>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <Eigen/Dense>

#include "../helper/helper.hpp"

namespace haptic
{
    struct Position_Params
    {
        hduVector3Dd previous_position_; // Предыдущее положение хаптика
        hduVector3Dd position_;          // Положение хаптика
        hduVector3Dd delta_position_;    // Смещение хаптика

        hduVector3Dd joint_angles_;      // Углы в джоинтах
        hduVector3Dd wrist_angles_;      // Углы в кулаке

        Eigen::Matrix<double,3,3> current_rot_;

        Eigen::Vector3d initial_pos_;
        Eigen::Vector3d temp_;
        Eigen::Vector3d current_pos_;

        const double radius_ = 0.5;
    };

    static int nButtons = 0;
    static bool btn_1;
    static bool btn_2;

    static clock_t last_time;
    static clock_t init_time;
    
    static Position_Params params;
    
    HDCallbackCode HDCALLBACK Callback(void *data);
}

#endif