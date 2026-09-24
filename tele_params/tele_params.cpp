#include "tele_params.hpp"

using namespace params;

params::TeleState::TeleState(int mode)
{
    switch (mode)
    {
    case 0:     // Обычный
        trans_factor_ = 1000;
        force_factor_ = 20;
        break;
    case 1:     // Грубый
        trans_factor_ = 100;
        force_factor_ = 30;
        break;
    case 2:     // Точный
        trans_factor_ = 10000;
        force_factor_ = 10;
        break;
    default:
        trans_factor_ = 1000;
        force_factor_ = 20;
        break;
    }

    current_rot_ << -1, 0, 0,
                    0, 1, 0,
                    0, 0, -1;

    // current_rot_ <<  -0.94, 0.0, 0.342,
    //                 0.0,  1.0,  0.0,
    //                 -0.342,  0.0, -0.94,

    // Задание начального положения
    initial_pos_ << 0.5, 0.0, 0.6;
    current_pos_ << 0.5, 0.0, 0.6;

    position_msg_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    forceVec_.set(0.0, 0.0, 0.0);
    
    server_.start();
    log_.start();

    init_time_ = std::chrono::steady_clock::now();
    last_time_ = init_time_;
    t_ = init_time_;
}

TeleState::~TeleState()
{
    server_.stop();
    log_.stop();
}

// ==================================================================================

void params::TeleState::setHapticState(const HapicState& haptic_state)
{
    
    btn_1 = (haptic_state.buttons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    btn_2 = (haptic_state.buttons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    position_ = haptic_state.position;
    joint_angles_ = haptic_state.joint_angles;
    wrist_angles_ = haptic_state.wrist_angles;

    time_ = std::chrono::steady_clock::now();
    // constexpr double y_step_delay_s = 5.0;
    // constexpr double y_step_duration_s = 3.0;
    // constexpr double y_step_target_m = 0.30;

    // const double elapsed_s = std::chrono::duration<double>(time_ - init_time_).count();
    // if (elapsed_s >= y_step_delay_s)
    // {
    //     previous_position_ = position_;

    //     if (!y_step_finished_)
    //     {
    //         const double ramp_s = std::min(elapsed_s - y_step_delay_s, y_step_duration_s);
    //         const double target_progress = y_step_target_m * ramp_s / y_step_duration_s;
    //         const double delta_y = target_progress - y_step_progress_;

    //         if (delta_y > 0.0)
    //         {
    //             current_pos_.y() += delta_y;
    //             y_step_progress_ = target_progress;

    //             position_msg_ << 0.0, delta_y, 0.0,
    //                             current_rot_(0,0), current_rot_(0,1), current_rot_(0,2),
    //                             current_rot_(1,0), current_rot_(1,1), current_rot_(1,2),
    //                             current_rot_(2,0), current_rot_(2,1), current_rot_(2,2);

    //             server_.setMsg(position_msg_);
    //             last_time_ = time_;
    //         }

    //         if (y_step_progress_ >= y_step_target_m)
    //         {
    //             y_step_finished_ = true;
    //             y_step_progress_ = y_step_target_m;
    //         }
    //     }

    //     return;
    // }

    // =================================================================================================================
    if ((btn_1) && (std::chrono::duration_cast<std::chrono::microseconds>(time_ - last_time_).count() >= 1000))    // =================================
    {   // =============================================================================================================

        delta_position_ = position_ - previous_position_;   // Смещение хаптика отнисительно предыдущего положения

        temp_ = current_pos_;   // Сохранение предыдущего положения

        // Изменение положения на смещение 
        current_pos_.x() = current_pos_.x() - delta_position_[2]/trans_factor_;
        current_pos_.y() = current_pos_.y() - delta_position_[0]/trans_factor_;
        current_pos_.z() = current_pos_.z() + delta_position_[1]/trans_factor_;
 
        // if (!checkPos())    // Проверка на выход за пределы разрешенной области
        // {   
        //     current_pos_ = temp_;   // Откат к предыдущиму положению
        //     std::cout << std::endl << "False" << std::endl;
        // }

        std::cout << "Текущая позиция:\n" << current_pos_.x() << "\t" << current_pos_.y() << "\t" << current_pos_.z() << std::endl;
        
        previous_position_ = position_;     // Сохраннение предыдущей позиции (для расчета смещения)

        std::cout << std::endl << "Текущая матрица:\n" << current_rot_ << std::endl;

        // ---------------------------------------------------------------------------------------------------

        position_msg_ << -delta_position_[2]/trans_factor_, -delta_position_[0]/trans_factor_, delta_position_[1]/trans_factor_, 
                                current_rot_(0,0), current_rot_(0,1), current_rot_(0,2),
                                current_rot_(1,0), current_rot_(1,1), current_rot_(1,2),
                                current_rot_(2,0), current_rot_(2,1), current_rot_(2,2);

        server_.setMsg(position_msg_);    //Отправка углов на контроллер

        // ---------------------------------------------------------------------------------------------------
        
        last_time_ = std::chrono::steady_clock::now();
        std::cout << "Время: " << std::chrono::duration_cast<std::chrono::microseconds>(last_time_ - time_).count() << std::endl << std::endl;

    }   // =============================================================================================================
    else if ((btn_2) && (std::chrono::duration_cast<std::chrono::microseconds>(time_ - last_time_).count() >= 1000)) // =============================
    {   // =============================================================================================================

        delta_position_ = position_ - previous_position_;       // Смещение хаптика отнисительно предыдущего положения

        temp_ = current_pos_;       // Сохранение предыдущего положения

        // Изменение положения на смещение 
        current_pos_.x() = current_pos_.x() - delta_position_[2]/trans_factor_*1;
        current_pos_.y() = current_pos_.y() - delta_position_[0]/trans_factor_*1;
        current_pos_.z() = current_pos_.z() + delta_position_[1]/trans_factor_*1;

        // if (!checkPos())    // Проверка на выход за пределы разрешенной области 
        // {   
        //     current_pos_ = temp_;   // Откат к предыдущиму положению
        //     std::cout << std::endl << "False" << std::endl;
        // }

        std::cout << "Текущая позиция:\n" << current_pos_.x() << "\t" << current_pos_.y() << "\t" << current_pos_.z() << std::endl;
        
        previous_position_ = position_;     // Сохраннение предыдущей позиции (для расчета смещения)

        current_rot_ = hapticFK(joint_angles_, wrist_angles_);      // Расчет ориентации

        std::cout << std::endl << "Текущая матрица:\n" << current_rot_ << std::endl;

        // ---------------------------------------------------------------------------------------------------

        position_msg_ << -delta_position_[2]/trans_factor_*1, -delta_position_[0]/trans_factor_*1, delta_position_[1]/trans_factor_*1, 
                                current_rot_(0,0), current_rot_(0,1), current_rot_(0,2),
                                current_rot_(1,0), current_rot_(1,1), current_rot_(1,2),
                                current_rot_(2,0), current_rot_(2,1), current_rot_(2,2);

        server_.setMsg(position_msg_);    //Отправка углов на контроллер

        // ---------------------------------------------------------------------------------------------------
        
        // std::cout << "Рассчитанные углы: " <<  server::eigenArrayToJson(params.thetta_).dump().c_str() << std::endl;

        last_time_ = std::chrono::steady_clock::now();
        std::cout << "Время: " << std::chrono::duration_cast<std::chrono::microseconds>(last_time_ - time_).count() << std::endl << std::endl;

    }   // =============================================================================================================
    else if (!(btn_1) && !(btn_2))   // ================================================================================
    {   // =============================================================================================================

        previous_position_ = position_;
        
    }
}

hduVector3Dd params::TeleState::getForceVector()
{
    if (server_.getMsg(obs_msg_))
    {
        
        // Масштабирование вектора силы
        forceVec_[2] = std::clamp(-obs_msg_[22]/force_factor_, -3., 3.);
        forceVec_[0] = std::clamp(-obs_msg_[23]/force_factor_, -3., 3.);
        forceVec_[1] = std::clamp( obs_msg_[24]/force_factor_, -3., 3.);

        // Сила
        // std::cout << "Force: " << forceVec[0] << "\t" << forceVec[1] << "\t" << forceVec[2] << std::endl;

        // const auto current_time = std::chrono::steady_clock::now();
        // const double command_time_us = static_cast<double>(
        //     std::chrono::duration_cast<std::chrono::microseconds>(time_ - init_time_).count());
        // const double current_time_us = static_cast<double>(
        //     std::chrono::duration_cast<std::chrono::microseconds>(current_time - init_time_).count());
        // const double button_state = btn_1 ? 1.0 : (btn_2 ? 2.0 : 0.0);

        // Eigen::Array<double, 9, 1> log_data;
        // log_data << command_time_us, current_time_us,
        //             current_pos_.x(), current_pos_.y(), current_pos_.z(),
        //             obs_msg_[7], obs_msg_[8], obs_msg_[9],
        //             button_state;
        // log_.setData(log_data);
    }

    return forceVec_;
}

void params::TeleState::setConnection()
{
    const char* name = "/my_shm2";

    int shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        return;
    }

    ftruncate(shm_fd, sizeof(bool));

    bool* ptr = (bool*)mmap(0, sizeof(bool), PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap");
        return;
    }

    // const char* message = "Привет от первого процесса!";
    // std::memcpy(ptr, message, strlen(message) + 1);
    *ptr = true;

    init_time_ = std::chrono::steady_clock::now();
    time_ = std::chrono::steady_clock::now();
    y_step_finished_ = false;
    y_step_progress_ = 0.0;

    munmap(ptr, sizeof(bool));
    close(shm_fd);
}

// ==================================================================================

bool params::TeleState::checkPos()
{
    return (current_pos_ - initial_pos_).norm() <= radius_;
}

Eigen::Matrix<double,3,3> params::TeleState::R(double theta, double alpha)         // Rotation matrix
{
    Eigen::Matrix<double,3,3> R;
    R << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                  0,             sin(alpha),             cos(alpha);
    return R;
}

Eigen::Matrix<double,3,3> params::TeleState::hapticFK(hduVector3Dd& joint_angles, hduVector3Dd& wrist_angles)      // Forvard kinematics
{
    Eigen::Matrix<double,3,3> rotation = Eigen::Matrix<double,3,3>::Identity(3,3);

    for (int8_t i = 0; i < 3; ++i)
    {
        rotation = rotation * R(dh_theta[i]+joint_angles[i]*dh_m[i], dh_alpha[i]);
    }
    for (int8_t i = 3; i < 6; ++i)
    {
        rotation = rotation * R(dh_theta[i]+wrist_angles[i-3]*dh_m[i], dh_alpha[i]);
    }
    return rotation;
}
