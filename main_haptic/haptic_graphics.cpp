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
    DeviceDisplayState state;
    gSchedulerCallback = hdScheduleAsynchronous(DeviceStateCallback, &state,
                          HD_MIN_SCHEDULER_PRIORITY);

    // Draw a sphere to represent the haptic cursor and the dynamic 
    // charge.
    static const float dynamicSphereColor[4] = { .8, .2, .2, .8 };
    drawSphere(pQuadObj, 
               state.position,
               dynamicSphereColor,
               sphereRadius);    

    gluDeleteQuadric(pQuadObj);
  
    glPopMatrix();
    glutSwapBuffers();                      
}

void graphics::handleIdle(void)
{
    glutPostRedisplay();

    if (!hdWaitForCompletion(gSchedulerCallback, HD_WAIT_CHECK_STATUS))
    {
        printf("The main scheduler callback has exited\n");
        printf("Press any key to quit.\n");
        return;
    }
}

void graphics::handleMenu(int ID)
{

    if (ID == 0)
    {
        exit(0);
    }
    // switch(ID) 
    // {
    //     case 0:
    //         exit(0);
    //         break;
    // }
}

HDCallbackCode HDCALLBACK graphics::DeviceStateCallback(void *pUserData)
{
    DeviceDisplayState *pDisplayState = 
        static_cast<DeviceDisplayState *>(pUserData);

    // double p;
    // double f;

    hdGetDoublev(HD_CURRENT_POSITION,pDisplayState->position);
    hdGetDoublev(HD_CURRENT_FORCE,pDisplayState->force);

    // execute this only once.
    return HD_CALLBACK_DONE;
}

