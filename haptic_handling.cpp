#include <iostream>
#include <cstdio>
#include <cassert>
#include <time.h>
#include <fstream>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <Eigen/Dense>

clock_t last_time;
clock_t init_time;

const double dh_theta[6] = {-M_PI/2, 0, M_PI/2, 0, 0, 0};                      // Параметры ДХ хаптика
const double dh_alpha[6] = {-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0};     // Параметры ДХ хаптика

Eigen::Matrix<double,3,3> R(double theta, double alpha);                 // Матрица поворота
Eigen::Matrix<double,3,3> FK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles);   // Прямая кинематика для хаптика (только матрица поворота)

Eigen::Matrix<double,3,3> current_rot;

HDCallbackCode HDCALLBACK TestCallback(void *data)
{

    hdBeginFrame(hdGetCurrentDevice());
   
    // Get the position of the device.
    hduVector3Dd position;
    hduVector3Dd joint_angles;
    hduVector3Dd wrist_angles;
    int nButtons = 0;
    bool btn_1;
    bool btn_2;

    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joint_angles);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, wrist_angles);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    btn_1 = (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    btn_2 = (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    if ((btn_1) && (((double)(clock() - last_time))/CLOCKS_PER_SEC*1000 >= 25))
    {
        printf("\nx) %f,\ny) %f,\nz) %f\n",position[0], position[1], position[2]);
        printf("a_j) %f,\nb_j) %f,\nc_j) %f\n",joint_angles[0]*180/M_PI, joint_angles[1]*180/M_PI, joint_angles[2]*180/M_PI);
        printf("a_w) %f,\nb_w) %f,\nc_w) %f\n\n",wrist_angles[0]*180/M_PI, wrist_angles[1]*180/M_PI, wrist_angles[2]*180/M_PI);
        current_rot = FK(joint_angles, wrist_angles);
        std::cout << std::endl << "Текущая матрица:\n" << current_rot << std::endl;
        last_time = clock();
    }

    if (btn_2)
    {
        return HD_CALLBACK_DONE;
    }

    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during main scheduler callback\n");

        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }        
    }

    return HD_CALLBACK_CONTINUE;
}

int main(int argc, char* argv[])
{

    last_time = clock();
    init_time = clock();

    char temp;
    HDErrorInfo error;
    // Initialize the default haptic device.
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        printf("\nAll closed\n");
        std::cout << temp;
        return -1;
    }

    // Start the servo scheduler and enable forces.
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        printf("\nAll closed\n");
        std::cout << temp;
        return -1;
    }
        
    // Application loop - schedule our call to the main callback.
    HDSchedulerHandle hSphereCallback = hdScheduleAsynchronous(
        TestCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    while (1)
    {
        if (!hdWaitForCompletion(hSphereCallback, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            printf("\nAll closed\n");   
            std::cout << temp;
            break;
        }
    }

    // For cleanup, unschedule our callbacks and stop the servo loop.
    hdStopScheduler();
    hdUnschedule(hSphereCallback);
    hdDisableDevice(hHD);
    printf("\nAll closed\n");

    return 0;
}

Eigen::Matrix<double,3,3> R(double theta, double alpha)         // Rotation matrix
{
    Eigen::Matrix<double,3,3> R;
    R << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                  0,             sin(alpha),             cos(alpha);
    return R;
}

Eigen::Matrix<double,3,3> FK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles)      // Forvard kinematics
{
    Eigen::Matrix<double,3,3> rotation = Eigen::Matrix<double,3,3>::Identity(3,3);

    for (int8_t i = 0; i < 3; ++i)
    {
        rotation = rotation * R(dh_theta[i]+joint_angles[i], dh_alpha[i]);
    }
    for (int8_t i = 3; i < 6; ++i)
    {
        rotation = rotation * R(dh_theta[i]+wrist_angles[i-3], dh_alpha[i]);
    }
    return rotation;
}