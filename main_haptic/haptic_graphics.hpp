#ifndef HAPTIC_GRAPHICS
#define HAPTIC_GRAPHICS

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

// #include <HD/hd.h>
#include <HL/hl.h>
#include <GL/glut.h>

#include<HLU/hlu.h>

#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

#include <Eigen/Dense>

#include "../helper/helper.hpp"
#include "../lib/my_lib_for_ik/facade/main_kinematic.hpp"
#include "../udp/udp_server.hpp"

namespace graphics
{

    struct DeviceDisplayState
    {
        HHD m_hHD;
        hduVector3Dd position;
        hduVector3Dd force;
    };  

    // Параметры кинематики и прочего
    struct Position_Params
    {
        hduVector3Dd previous_position_; // Предыдущее положение хаптика
        hduVector3Dd position_;          // Положение хаптика
        hduVector3Dd delta_position_;    // Смещение хаптика

        hduVector3Dd joint_angles_;      // Углы в джоинтах
        hduVector3Dd wrist_angles_;      // Углы в кулаке

        // hduQuaternion quat_;
        HDdouble transform_[16];

        Eigen::Matrix<double,3,3> current_rot_; // Матрица ориентации

        Eigen::Vector3d initial_pos_;   // Начальное положение
        Eigen::Vector3d temp_;          // Временный вектор
        Eigen::Vector3d current_pos_;   // Текущее положение

        const double radius_ = 0.5;     // Радиус разрешенной области

        Kinematic kinematic_ = Kinematic(new DrakeKinematic("../robots/iiwa.urdf"));    // Решатель кинематики
        // Kinematic kinematic_ = Kinematic(new KDLKinematic());

        Eigen::Array<double, 7,1> thetta_;          // Рассчитанные углы в джоинтах

        Eigen::Array<double, 14,1> torque_msg_;     // Сообщение от контроллера

        Eigen::Array<double, 7,1> current_kuka_thetta_;     // Реальное положение куки 
        Eigen::Array<double, 7,1> current_kuka_torque_;     // Торки в джоинтах куки

        Eigen::Array<double, 6,1> force_;   // Вектор силы и моменты

    };

    static HHD ghHD = HD_INVALID_HANDLE;
    static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

    // ================================================================================

    static double sphereRadius = 5.0;   // Радиус сферки на экране

    static bool first = true;   // Первый запуск
    static int state;           // Статус решения обратной кинематики

    static Eigen::Matrix<double,3,3> ad_rot_;

    static int nButtons = 0;
    static bool btn_1;          // Нажата 1ая кнопка
    static bool btn_2;          // Нажата 2ая кнопка

    static clock_t last_time;   
    static clock_t init_time;

    static clock_t t;
    
    static Position_Params params;  // Параметры куки

    static server::UDPServer server("127.0.0.1", 8080, "127.0.0.1", 8081);  // UDP

    // ================================================================================

    void initGlut(int argc, char* argv[]);
    void initGraphics(const hduVector3Dd &LLB, const hduVector3Dd &TRF);
    void setupGraphicsState();
    void drawAxes(double axisLength);
    void drawSphere(GLUquadricObj* pQuadObj, 
                const hduVector3Dd &position,
                const float color[4],
                double sphereRadius);
    void drawForceVector(GLUquadricObj* pQuadObj,
                     const hduVector3Dd &position,
                     const hduVector3Dd &forceVector,
                     double arrowThickness);
    void displayFunction(void);

    HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData);
    void handleIdle(void);
    void handleMenu(int ID);

    hduVector3Dd forceField(Eigen::Array<double, 7,1> thetta_, Eigen::Array<double, 7,1> torque_);
    HDCallbackCode HDCALLBACK Callback(void *data);
    void HapticControl();

    void exitHandler();

}

#endif