#include "main_haptic/haptic_handling.hpp"
#include "main_haptic/haptic_graphics.hpp"

int main(int argc, char* argv[])
{

    haptic::last_time = clock();
    haptic::init_time = clock();

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
        haptic::Callback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    graphics::initGlut(argc, argv);

    // Get the workspace dimensions.
    HDdouble maxWorkspace[6];
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);

    // Low/left/back point of device workspace.
    hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
    // Top/right/front point of device workspace.
    hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
    
    graphics::initGraphics(LLB, TRF);

    glutMainLoop();

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

