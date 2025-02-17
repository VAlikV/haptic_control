#include "haptic_graphics.hpp"

using namespace graphics;

void graphics::initGlut(int argc, char* argv[])
{
    // Initialize GLUT.
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("GLUT/Coulomb Forces Demo");

    // Setup GLUT callbacks.
    glutDisplayFunc(displayFunction); 
    glutIdleFunc(handleIdle);

    // Setup GLUT popup menu.
    glutCreateMenu(handleMenu); 
    glutAddMenuEntry("Reverse Charge", 1);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
    glutAddMenuEntry("Quit", 0);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::initGraphics(const hduVector3Dd &LLB, const hduVector3Dd &TRF)
{
    // Setup perspective projection.
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity();

    HDdouble centerScreen[3];
    centerScreen[0] = (TRF[0] + LLB[0])/2.0;
    centerScreen[1] = (TRF[1] + LLB[1])/2.0;
    centerScreen[2] = (TRF[2] + LLB[2])/2.0;

    HDdouble screenDims[3];
    screenDims[0] = TRF[0] - LLB[0];
    screenDims[1] = TRF[1] - LLB[1];
    screenDims[2] = TRF[2] - LLB[2];

    HDdouble maxDimXY = screenDims[0] > screenDims[1] ? 
        screenDims[0] : screenDims[1];
    HDdouble maxDim = maxDimXY > screenDims[2] ? 
        maxDimXY : screenDims[2];
    maxDim /= 2.0;

    glOrtho(centerScreen[0]-maxDim, centerScreen[0]+maxDim, 
            centerScreen[1]-maxDim, centerScreen[1]+maxDim,
            centerScreen[2]-maxDim, centerScreen[2]+maxDim);
    
    glShadeModel(GL_SMOOTH);

    // Setup model transformations.
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::setupGraphicsState()
{
    glShadeModel(GL_SMOOTH);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHT_MODEL_TWO_SIDE);
    glShadeModel(GL_SMOOTH);
    
    GLfloat lightZeroPosition[] = { 10.0, 4.0, 100.0, 0.0 };
    GLfloat lightZeroColor[] = { 0.6, 0.6, 0.6, 1.0 }; // green-tinted.
    GLfloat lightOnePosition[] = { -1.0, -2.0, -100.0, 0.0 };
    GLfloat lightOneColor[] = { 0.6, 0.6, 0.6, 1.0 }; // red-tinted.
    
    GLfloat light_ambient[] = { 0.8, 0.8, 0.8, 1.0 }; // White diffuse light.
    GLfloat light_diffuse[] = { 0.0, 0.0, 0.0, 1.0 }; // White diffuse light.
    GLfloat light_position[] = { 0.0, 0.0, 100.0, 1.0 }; // Infinite light loc.
    
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::drawAxes(double axisLength)
{
    glDisable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glLineWidth(2.0);
    
    glBegin(GL_LINES);
    for (int i = 0; i < 3; i++) 
    {
        float color[3] = { 0, 0, 0 };
        color[i] = 1.0;
        glColor3fv(color);
        
        float vertex[3] = {0, 0, 0};
        vertex[i] = axisLength;
        glVertex3fv(vertex);
        glVertex3f(0, 0, 0);
    } 
    glEnd();
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::drawSphere(GLUquadricObj* pQuadObj, 
                const hduVector3Dd &position,
                const float color[4],
                double sphereRadius)
{
    glMatrixMode(GL_MODELVIEW); 
    glPushMatrix();
    glEnable(GL_LIGHTING);
    glColor4fv(color);
    glTranslatef(position[0], position[1], position[2]);
    gluSphere(pQuadObj, sphereRadius, 20, 20); 
    glPopMatrix();
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::drawForceVector(GLUquadricObj* pQuadObj,
                     const hduVector3Dd &position,
                     const hduVector3Dd &forceVector,
                     double arrowThickness)
{
    glDisable(GL_LIGHTING);
    
    glPushMatrix();

    glTranslatef(position[0], position[1], position[2]);

    // Change the force magnitude/direction by rotating the force vector.
    // Calculate the rotation angle.
    hduVector3Dd unitForceVectorAxis = normalize(forceVector);
    hduVector3Dd zAxis( 0.0, 0.0, 1.0 );
    hduVector3Dd toolRotAxis = zAxis.crossProduct(unitForceVectorAxis);
        
    double toolRotAngle = acos(unitForceVectorAxis[2]);
    hduMatrix rotMatrix = hduMatrix::createRotation(toolRotAxis, 
                                                    toolRotAngle);

    double rotVals[4][4];
    rotMatrix.get(rotVals);
    glMultMatrixd((double*) rotVals);

    // The force arrow: composed of a cylinder and a cone.
    glColor3f( 0.2, 0.7, 0.2 );
    
    double strength = forceVector.magnitude();
    
    // Draw arrow shaft.
    gluCylinder(pQuadObj,arrowThickness, arrowThickness, strength, 16, 2); 
    glTranslatef(0, 0, strength);
    glColor3f(0.2, 0.8, 0.3);
    
    // Draw arrow head.
    gluCylinder(pQuadObj, arrowThickness*2, 0.0, strength*.15, 16, 2); 
    
    glPopMatrix();
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

HDCallbackCode HDCALLBACK graphics::DeviceStateCallback(void *pUserData)
{
    DeviceDisplayState *pDisplayState = 
        static_cast<DeviceDisplayState *>(pUserData);

    hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->position);
    hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);

    // execute this only once.
    return HD_CALLBACK_DONE;
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::displayFunction(void)
{
    // Setup model transformations.
    glMatrixMode(GL_MODELVIEW); 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();

    setupGraphicsState();
    drawAxes(sphereRadius*3.0);

    // Draw the fixed sphere.
    static const hduVector3Dd fixedSpherePosition(0, 0, 0);
    static const float fixedSphereColor[4] = {.2, .8, .8, .8};
    GLUquadricObj* pQuadObj = gluNewQuadric();
    drawSphere(pQuadObj, fixedSpherePosition, fixedSphereColor, sphereRadius);

    // Get the current position of end effector.
    // DeviceDisplayState state;
    // hdScheduleSynchronous(DeviceStateCallback, &state,
    //                       HD_MIN_SCHEDULER_PRIORITY);

    // Draw a sphere to represent the haptic cursor and the dynamic 
    // charge.

    hduVector3Dd position((params.current_pos_.x() - params.initial_pos_.x())*100,
                            (params.current_pos_.y() - params.initial_pos_.y())*100,
                            (params.current_pos_.z() - params.initial_pos_.z())*100);

    static const float dynamicSphereColor[4] = { .8, .8, .2, .8 };
    drawSphere(pQuadObj, 
               position,
               dynamicSphereColor,
               sphereRadius);    

    // Create the force vector.
    // hduVector3Dd forceVector = 400.0 * forceField(state.position);
    
    // drawForceVector(pQuadObj,
    //                 state.position,
    //                 forceVector,
    //                 sphereRadius*.1);

    // gluDeleteQuadric(pQuadObj);
  
    glPopMatrix();
    glutSwapBuffers();                      
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::handleIdle(void)
{
    glutPostRedisplay();

    if (!hdWaitForCompletion(gSchedulerCallback, HD_WAIT_CHECK_STATUS))
    {
        printf("The main scheduler callback has exited\n");
        printf("Press any key to quit.\n");
        getchar();
        exit(-1);
    }
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::handleMenu(int ID)
{
    switch(ID) 
    {
        case 0:
            exit(0);
            break;
    }
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

hduVector3Dd graphics::forceField(Eigen::Array<double, 7,1> thetta_, Eigen::Array<double, 7,1> torque_)
{
    
    hduVector3Dd forceVec(0,0,0);

    // params.force_ = params.kinematic_.getForce(params.current_kuka_thetta_, params.current_kuka_torque_);

    // std::cout << "Force: " << params.force_.transpose() << std::endl;
    
    // if two charges overlap...
    // if(dist < sphereRadius*2.0) 
    // {
    //     // Attract the charge to the center of the sphere.
    //     forceVec =  -0.1*pos;
    // }
    // else
    // {
    //     hduVector3Dd unitPos = normalize(pos);
    //     forceVec = -1200.0*unitPos/(dist*dist);
    // }
    // forceVec *= charge;
    return forceVec;
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

HDCallbackCode HDCALLBACK graphics::Callback(void *data)
{
    HHD hHD = hdGetCurrentDevice();

    hdBeginFrame(hHD);

    hdGetDoublev(HD_CURRENT_POSITION, params.position_);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, params.joint_angles_);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, params.wrist_angles_);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &(nButtons));
    
    btn_1 = (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    btn_2 = (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    if (first)
    {
        // params.current_rot_ = kinematics_helper::FK(params.joint_angles_, params.wrist_angles_);
        params.current_rot_ << -1, 0, 0,
                                0, 1, 0,
                                0, 0, -1;

        params.initial_pos_ << 0.5, 0.0, 0.7;
        params.current_pos_ << 0.5, 0.0, 0.7;

        first = false;
    }

    if ((btn_1) && (((double)(clock() - last_time))/CLOCKS_PER_SEC*1000 >= 25))
    {

        t = clock();

        params.delta_position_ = params.position_ - params.previous_position_;

        params.temp_ = params.current_pos_;

        params.current_pos_.x() = params.current_pos_.x() - params.delta_position_[2]/1000;
        params.current_pos_.y() = params.current_pos_.y() - params.delta_position_[0]/1000;
        params.current_pos_.z() = params.current_pos_.z() + params.delta_position_[1]/1000;

        if (!kinematics_helper::checkPos(params.current_pos_, params.initial_pos_, params.radius_))
        {
            params.current_pos_ = params.temp_;
            std::cout << std::endl << "False" << std::endl;
        }

        std::cout << "Текущая позиция:\n" << params.current_pos_.x() << "\t" << params.current_pos_.y() << "\t" << params.current_pos_.z() << std::endl;

        params.previous_position_ = params.position_;

        // params.current_rot_ = kinematics_helper::FK(params.joint_angles_, params.wrist_angles_);
        std::cout << std::endl << "Текущая матрица:\n" << params.current_rot_ << std::endl;

        t = clock();
        
        state = params.kinematic_.IK(params.current_rot_, params.current_pos_);
        params.thetta_ = params.kinematic_.getQRad();
        server.setMsg(params.thetta_);

        std::cout << "Статус: " << state << std::endl;
        std::cout << "Рассчитанные углы: " <<  params.thetta_.transpose()*180/M_PI << std::endl;
        
        // std::cout << "Рассчитанные углы: " <<  server::eigenArrayToJson(params.thetta_).dump().c_str() << std::endl;

        last_time = clock();

        std::cout << "Время: " << ((double)(clock() - t))/CLOCKS_PER_SEC*1000 << std::endl << std::endl;
    }
    else if (!(btn_1))
    {
        params.previous_position_ = params.position_;
    }

    if (server.getMsg(params.torque_msg_))
    {
        // std::cout << params.torque_msg_.transpose() << std::endl;
        params.current_kuka_thetta_ << params.torque_msg_[0], params.torque_msg_[1], params.torque_msg_[2], params.torque_msg_[3], params.torque_msg_[4], params.torque_msg_[5], params.torque_msg_[6]; 
        params.current_kuka_torque_ << params.torque_msg_[7], params.torque_msg_[8], params.torque_msg_[9], params.torque_msg_[10], params.torque_msg_[11], params.torque_msg_[12], params.torque_msg_[13]; 

        params.force_ = params.kinematic_.getForce(params.current_kuka_thetta_, params.current_kuka_torque_);

        hduVector3Dd forceVec;

        forceVec[0] = params.force_[0];
        forceVec[1] = params.force_[1];
        forceVec[2] = params.force_[2];

        // forceVec = forceField(pos);
        hdSetDoublev(HD_CURRENT_FORCE, forceVec);

    };      // Чтение пришедших по UDP данных
    
    hdEndFrame(hHD);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during scheduler callback");
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::HapticControl()
{
    server.start();

    gSchedulerCallback = hdScheduleAsynchronous(
        Callback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }


    glutMainLoop(); // Enter GLUT main loop.
}

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

void graphics::exitHandler()
{
    hdStopScheduler();
    hdUnschedule(gSchedulerCallback);

    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
        ghHD = HD_INVALID_HANDLE;
    }
}

