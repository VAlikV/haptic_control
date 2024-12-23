#include "haptic_handling.hpp"

using namespace haptic;
using namespace kinematics_helper;

HDCallbackCode HDCALLBACK haptic::Callback(void *data)
{
hdBeginFrame(hdGetCurrentDevice());
   
    // Get the position of the device.

    hdGetDoublev(HD_CURRENT_POSITION, params.position_);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, params.joint_angles_);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, params.wrist_angles_);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &(nButtons));
    
    btn_1 = (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    btn_2 = (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    if ((btn_1) && (((double)(clock() - last_time))/CLOCKS_PER_SEC*1000 >= 25))
    {
        // printf("\nx) %f,\ny) %f,\nz) %f\n",this_ptr->position_[0], this_ptr->position_[1], this_ptr->position_[2]);
        // printf("a_j) %f,\nb_j) %f,\nc_j) %f\n",this_ptr->joint_angles_[0]*180/M_PI, this_ptr->joint_angles_[1]*180/M_PI, this_ptr->joint_angles_[2]*180/M_PI);
        // printf("a_w) %f,\nb_w) %f,\nc_w) %f\n\n",this_ptr->wrist_angles_[0]*180/M_PI, this_ptr->wrist_angles_[1]*180/M_PI, this_ptr->wrist_angles_[2]*180/M_PI);
        
        params.delta_position_ = params.position_ - params.previous_position_;

        params.temp_ = params.current_pos_;

        params.current_pos_.x() = params.current_pos_.x() + params.delta_position_[0]/100;
        params.current_pos_.y() = params.current_pos_.y() + params.delta_position_[1]/100;
        params.current_pos_.z() = params.current_pos_.z() + params.delta_position_[2]/100;

        if (!checkPos(params.current_pos_, params.initial_pos_, params.radius_))
        {
            params.current_pos_ = params.temp_;
            std::cout << std::endl << "False" << std::endl;
        }

        std::cout << "Текущая позиция:\n" << params.current_pos_.x() << "\t" << params.current_pos_.y() << "\t" << params.current_pos_.z() << std::endl;

        params.previous_position_ = params.position_;

        // int state = params.kinematic_->IK(this_ptr->current_rot_, this_ptr->current_pos_);

        last_time = clock();
    }
    else if (!(btn_1))
    {
        params.previous_position_ = params.position_;
    }

    if ((btn_2) && (((double)(clock() - last_time))/CLOCKS_PER_SEC*1000 >= 25))
    {

        params.current_rot_ = FK(params.joint_angles_, params.wrist_angles_);
        std::cout << std::endl << "Текущая матрица:\n" << params.current_rot_ << std::endl;

        last_time = clock();
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
