#include "main_haptic.hpp"

MainHaptic::MainHaptic()
{
    printf("Start");

    initial_pos_ << 0, -0.7, 0.45;
    current_pos_ << 0, -0.7, 0.45;

    current_rot_ << -0.172954,     0.98493, 7.44406e-05,
                    0.630024,    0.11069,   -0.768646,
                    -0.757071,   -0.132893,   -0.639674;

    KDLKinematic* KDL_solver = new KDLKinematic();

    kinematic_ = new Kinematic(KDL_solver);
}

MainHaptic::~MainHaptic()
{
    delete kinematic_;
    kinematic_ = nullptr;
}

HDCallbackCode HDCALLBACK MainHaptic::Callback(void *data)
{
    MainHaptic* this_ptr = (MainHaptic *)data;

    hdBeginFrame(hdGetCurrentDevice());
   
    // Get the position of the device.

    hdGetDoublev(HD_CURRENT_POSITION, this_ptr->position_);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, this_ptr->joint_angles_);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, this_ptr->wrist_angles_);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &(this_ptr->nButtons_));
    
    this_ptr->btn_1_ = (this_ptr->nButtons_ & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    this_ptr->btn_2_ = (this_ptr->nButtons_ & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    if ((this_ptr->btn_1_) && (((double)(clock() - this_ptr->last_time_))/CLOCKS_PER_SEC*1000 >= 25))
    {
        // printf("\nx) %f,\ny) %f,\nz) %f\n",this_ptr->position_[0], this_ptr->position_[1], this_ptr->position_[2]);
        // printf("a_j) %f,\nb_j) %f,\nc_j) %f\n",this_ptr->joint_angles_[0]*180/M_PI, this_ptr->joint_angles_[1]*180/M_PI, this_ptr->joint_angles_[2]*180/M_PI);
        // printf("a_w) %f,\nb_w) %f,\nc_w) %f\n\n",this_ptr->wrist_angles_[0]*180/M_PI, this_ptr->wrist_angles_[1]*180/M_PI, this_ptr->wrist_angles_[2]*180/M_PI);
        
        this_ptr->delta_position_ = this_ptr->position_ - this_ptr->previous_position_;

        this_ptr->temp_ = this_ptr->current_pos_;

        this_ptr->current_pos_.x() = this_ptr->current_pos_.x() + this_ptr->delta_position_[0]/100;
        this_ptr->current_pos_.y() = this_ptr->current_pos_.y() + this_ptr->delta_position_[1]/100;
        this_ptr->current_pos_.z() = this_ptr->current_pos_.z() + this_ptr->delta_position_[2]/100;

        if (!this_ptr->checkPos(this_ptr->current_pos_))
        {
            this_ptr->current_pos_ = this_ptr->temp_;
            std::cout << std::endl << "False" << std::endl;
        }

        std::cout << "Текущая позиция:\n" << this_ptr->current_pos_.x() << "\t" << this_ptr->current_pos_.y() << "\t" << this_ptr->current_pos_.z() << std::endl;

        this_ptr->previous_position_ = this_ptr->position_;

        int state = this_ptr->kinematic_->IK(this_ptr->current_rot_, this_ptr->current_pos_);

        

        this_ptr->last_time_ = clock();
    }
    else if (!(this_ptr->btn_1_))
    {
        this_ptr->previous_position_ = this_ptr->position_;
    }

    if ((this_ptr->btn_2_) && (((double)(clock() - this_ptr->last_time_))/CLOCKS_PER_SEC*1000 >= 25))
    {

        this_ptr->current_rot_ = this_ptr->FK(this_ptr->joint_angles_, this_ptr->wrist_angles_);
        std::cout << std::endl << "Текущая матрица:\n" << this_ptr->current_rot_ << std::endl;

        this_ptr->last_time_ = clock();
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

int MainHaptic::hapticInit()
{
    last_time_ = clock();
    init_time_ = clock();

    HDErrorInfo error;

    // Initialize the default haptic device.
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        printf("\nAll closed\n");
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
        return -1;
    }

    // auto bind_callback = std::bind(&MainHaptic::Callback,this,std::placeholders::_1);
        
    // Application loop - schedule our call to the main callback.
    HDSchedulerHandle hSphereCallback = hdScheduleAsynchronous(
        Callback, (void *)this, HD_DEFAULT_SCHEDULER_PRIORITY);

    while (1)
    {
        if (!hdWaitForCompletion(hSphereCallback, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            printf("\nAll closed\n");   
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

Eigen::Matrix<double,3,3> MainHaptic::R(double theta, double alpha)         // Rotation matrix
{
    Eigen::Matrix<double,3,3> R;
    R << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                  0,             sin(alpha),             cos(alpha);
    return R;
}

Eigen::Matrix<double,3,3> MainHaptic::FK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles)      // Forvard kinematics
{
    Eigen::Matrix<double,3,3> rotation = Eigen::Matrix<double,3,3>::Identity(3,3);

    joint_angles[0] = -joint_angles[0];
    wrist_angles[0] = -wrist_angles[0];

    for (int8_t i = 0; i < 3; ++i)
    {
        rotation = rotation * R(dh_theta_[i]+joint_angles[i], dh_alpha_[i]);
    }
    for (int8_t i = 3; i < 6; ++i)
    {
        rotation = rotation * R(dh_theta_[i]+wrist_angles[i-3], dh_alpha_[i]);
    }
    return rotation;
}

bool MainHaptic::checkPos(Eigen::Vector3d current)
{
    if ((current - initial_pos_).norm() <= radius_)
    {
        return true;
    }
    else
    {
        return false;
    }
}