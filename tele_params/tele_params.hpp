#ifndef TELEPARAMS
#define TELEPARAMS

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

#include <HD/hd.h>
#include <HL/hl.h>
#include <HLU/hlu.h>

#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

#include <Eigen/Dense>

#include "../ik/drake_kinematic.hpp"
#include "../udp/udp_server.hpp"
#include "../logger/logger.hpp"

#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>

using namespace iiwa_kinematics;

namespace params
{
    struct HapicState
    {
        hduVector3Dd position;          // Положение хаптика
        hduVector3Dd joint_angles;      // Углы в джоинтах
        hduVector3Dd wrist_angles;      // Углы в кулаке
        int buttons = 0;                // Состояние кнопки
        hduVector3Dd force;
    };  
    
    class TeleState
    {
    private:
        double trans_factor_ = 1000;
        double force_factor_ = 20;

        hduVector3Dd position_;          // Положение хаптика
        hduVector3Dd previous_position_; // Предыдущее положение хаптика
        hduVector3Dd delta_position_;    // Смещение хаптика

        hduVector3Dd joint_angles_;      // Углы в джоинтах
        hduVector3Dd wrist_angles_;      // Углы в кулаке

        Eigen::Matrix<double,3,3> current_rot_; // Матрица ориентации

        Eigen::Vector3d initial_pos_;   // Начальное положение
        Eigen::Vector3d temp_;          // Временный вектор
        Eigen::Vector3d current_pos_;   // Текущее положение

        const double radius_ = 0.5;     // Радиус разрешенной области

        // ================================================================================
        // ================================================================================    
        // ================================================================================    

        DrakeKinematic kinematic_ = DrakeKinematic("../robots/iiwa.urdf");    // Решатель кинематики

        Eigen::Array<double, 7,1> thetta_;          // Рассчитанные углы в джоинтах

        Eigen::Array<double, 14,1> torque_msg_;     // Сообщение от контроллера

        double dt_ = 1;
        bool first_msg_ = true;
        std::chrono::steady_clock::time_point last_msg_;   
        Eigen::Array<double, 7,1> previous_kuka_thetta_;     // Предыдущее положение куки 
        Eigen::Array<double, 7,1> current_kuka_thetta_;     // Реальная скорость куки 
        Eigen::Array<double, 7,1> previous_kuka_d_thetta_;     // Предыдущая скорость куки 
        Eigen::Array<double, 7,1> current_kuka_d_thetta_;     // Реальная скорость куки 
        Eigen::Array<double, 7,1> current_kuka_dd_thetta_;     // Реальное ускорение куки 

        Eigen::Array<double, 7,1> current_kuka_torque_;     // Торки в джоинтах куки
        Eigen::Array<double, 7,1> inertia_kuka_torque_;     

        Eigen::Array<double, 6,1> force_;   // Вектор силы и моменты

        hduVector3Dd forceVec_;

        // ================================================================================    
        // ================================================================================    
        // ================================================================================    

        int state_;           // Статус решения обратной кинематики

        bool btn_1;          // Нажата 1ая кнопка
        bool btn_2;          // Нажата 2ая кнопка

        std::chrono::steady_clock::time_point last_time_;   
        std::chrono::steady_clock::time_point init_time_;

        std::chrono::steady_clock::time_point t_;
        std::chrono::steady_clock::time_point time_;

        server::UDPServer server_ = server::UDPServer("127.0.0.1", 8080, "127.0.0.1", 8081);  // UDP

        int nn_ = 0;
        Eigen::Array<double, 9,1> data_time_;        // Для времени
        Eigen::Array<double, 9,1> data_joints_;        // Углы в джоинтах для сохранения
        logger::FileLogger log_ = logger::FileLogger("HapticLogs.txt");

        // ================================================================================    
        // ================================================================================    
        // ================================================================================  

        const double dh_m[6] = {-1, 1, 1, -1, -1, 1};                // Множители хаптика
        const double dh_theta[6] = {M_PI, 0, 0, 0, -M_PI/2, 0};                // Параметры ДХ хаптика
        const double dh_alpha[6] = {M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0};     // Параметры ДХ хаптика

        Eigen::Matrix<double,3,3> R(double theta, double alpha);

        Eigen::Matrix<double,3,3> hapticFK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles);

    public:
        TeleState(int mode = 0);
        ~TeleState();
        void setHapticState(const HapicState& haptic_state);
        hduVector3Dd getForceVector();
        void setConnection();

        bool checkPos();
    };   

    // ================================================================================    

}

#endif