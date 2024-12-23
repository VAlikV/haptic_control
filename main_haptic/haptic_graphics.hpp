#ifndef HAPTIC_GRAPHICS
#define HAPTIC_GRAPHICS

#include <stdio.h>
#include <stdlib.h>

#include <HD/hd.h>
# include <GL/glut.h>

#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <Eigen/Dense>

#include "../helper/helper.hpp"

namespace graphics
{

    struct DeviceDisplayState
    {
        HHD m_hHD;
        hduVector3Dd position;
        hduVector3Dd force;
    };

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

    static HHD ghHD = HD_INVALID_HANDLE;
    static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

    static double sphereRadius = 5.0;

    static int nButtons = 0;
    static bool btn_1;
    static bool btn_2;

    static clock_t last_time;
    static clock_t init_time;
    
    static Position_Params params;

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

    hduVector3Dd forceField(hduVector3Dd pos);
    HDCallbackCode HDCALLBACK Callback(void *data);
    void HapticControl();

    void exitHandler();

}

#endif