// #include "main_haptic/haptic_handling.hpp"
#include "tele_params/tele_params.hpp"

/*******************************************************************************
 Haptic sphere callback.  
 The sphere is oriented at 0,0,0 with radius 40, and provides a repelling force 
 if the device attempts to penetrate through it. 
*******************************************************************************/
HDCallbackCode HDCALLBACK FrictionlessSphereCallback(void *data)
{
    hdBeginFrame(hdGetCurrentDevice());
   
    // My code

    // End code

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

/******************************************************************************
 main function
 Initializes the device, creates a callback to handle sphere forces, terminates
 upon key press.
******************************************************************************/
int main(int argc, char* argv[])
{
    // My code

    // End code

    HDErrorInfo error;
    // Initialize the default haptic device.
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    // Start the servo scheduler and enable forces.
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
        
    // Application loop - schedule our call to the main callback.
    HDSchedulerHandle hSphereCallback = hdScheduleAsynchronous(
        FrictionlessSphereCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    printf("Sphere example.\n");
    printf("Move the device around to feel a frictionless sphere\n\n");
    printf("Press any key to quit.\n\n");

    while (!_kbhit())
    {
        if (!hdWaitForCompletion(hSphereCallback, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            break;
        }
    }

    // For cleanup, unschedule our callbacks and stop the servo loop.
    hdStopScheduler();
    hdUnschedule(hSphereCallback);
    hdDisableDevice(hHD);

    return 0;
}

/*****************************************************************************/

