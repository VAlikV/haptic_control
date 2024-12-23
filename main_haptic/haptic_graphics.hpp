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

namespace graphics
{

    struct DeviceDisplayState
    {
        HHD m_hHD;
        hduVector3Dd position;
        hduVector3Dd force;
    };

    static HHD ghHD = HD_INVALID_HANDLE;
    static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

    static double sphereRadius;

    void initGlut(int argc, char* argv[]);
    void initGraphics(const hduVector3Dd &LLB, const hduVector3Dd &TRF);
    void setupGraphicsState();
    void drawAxes(double axisLength);
    void drawSphere(GLUquadricObj* pQuadObj, 
                const hduVector3Dd &position,
                const float color[4],
                double sphereRadius);
    void displayFunction(void);

    HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData);
    void handleIdle(void);
    void handleMenu(int ID);

}

#endif