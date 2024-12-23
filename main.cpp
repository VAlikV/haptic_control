// #include "main_haptic/haptic_handling.hpp"
#include "main_haptic/haptic_graphics.hpp"

int main(int argc, char* argv[])
{
    graphics::last_time = clock();
    graphics::init_time = clock();

    HDErrorInfo error;

    printf("Starting application\n");
    
    atexit(graphics::exitHandler);

    // Initialize the device.  This needs to be called before any other
    // actions on the device are performed.
    graphics::ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));
    
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }
    
    graphics::initGlut(argc, argv);

    // Get the workspace dimensions.
    HDdouble maxWorkspace[6];
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);

    // Low/left/back point of device workspace.
    hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
    // Top/right/front point of device workspace.
    hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
    graphics::initGraphics(LLB, TRF);

    // Application loop.
    graphics::HapticControl();

    printf("Done\n");
    return 0;
}

