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

#include "../udp/udp_server.hpp"
#include "../logger/logger.hpp"

#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>

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

        Eigen::Array<double, 12,1> position_msg_;   // Рассчитанные углы в джоинтах
        Eigen::Array<double,6,1> force_msg_;        // Сообщение от контроллера

        hduVector3Dd forceVec_;

        // ================================================================================    
        // ================================================================================    
        // ================================================================================    

        bool btn_1;          // Нажата 1ая кнопка
        bool btn_2;          // Нажата 2ая кнопка

        std::chrono::steady_clock::time_point last_time_;   
        std::chrono::steady_clock::time_point init_time_;

        std::chrono::steady_clock::time_point t_;
        std::chrono::steady_clock::time_point time_;

        server::UDPServer<6,12> server_ = server::UDPServer<6,12>("127.0.0.1", 8080, "127.0.0.1", 8081);  // UDP

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