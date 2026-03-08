#include "tele_params.hpp"

using namespace params;

params::TeleState::TeleState(int mode)
// kinematic_(DrakeKinematic("../robots/iiwa.urdf")),
// server_(server::UDPServer("127.0.0.1", 8080, "127.0.0.1", 8081))
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

    // Задание начального положения
    initial_pos_ << 0.5, 0.0, 0.6;
    current_pos_ << 0.5, 0.0, 0.6;

    data_time_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    data_joints_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    server_.start();
    log_.start();

    // init_time_ = clock();
    last_time_ = std::chrono::steady_clock::now();
    t_ = std::chrono::steady_clock::now();
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
    // =================================================================================================================
    if ((btn_1) && (std::chrono::duration_cast<std::chrono::microseconds>(time_ - last_time_).count() >= 25000))    // =================================
    {   // =============================================================================================================

        delta_position_ = position_ - previous_position_;   // Смещение хаптика отнисительно предыдущего положения

        temp_ = current_pos_;   // Сохранение предыдущего положения

        // Изменение положения на смещение 
        current_pos_.x() = current_pos_.x() - delta_position_[2]/trans_factor_;
        current_pos_.y() = current_pos_.y() - delta_position_[0]/trans_factor_;
        current_pos_.z() = current_pos_.z() + delta_position_[1]/trans_factor_;
 
        if (!checkPos())    // Проверка на выход за пределы разрешенной области
        {   
            current_pos_ = temp_;   // Откат к предыдущиму положению
            std::cout << std::endl << "False" << std::endl;
        }

        std::cout << "Текущая позиция:\n" << current_pos_.x() << "\t" << current_pos_.y() << "\t" << current_pos_.z() << std::endl;
        
        previous_position_ = position_;     // Сохраннение предыдущей позиции (для расчета смещения)

        std::cout << std::endl << "Текущая матрица:\n" << current_rot_ << std::endl;

        // ---------------------------------------------------------------------------------------------------

        t_ = std::chrono::steady_clock::now();
        data_time_[1] = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t_ - init_time_).count());
        log_.setData(data_time_);

        // Решение обратной кинематики
        kinematic_.setPositionVector(current_pos_);
        kinematic_.setRotationMatrix(current_rot_);
        state_ = kinematic_.IK();
        thetta_ = kinematic_.getQ();

        t_ = std::chrono::steady_clock::now();
        data_time_[1] = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t_ - init_time_).count());
        log_.setData(data_time_);

        // ---------------------------------------------------------------------------------------------------

        server_.setMsg(thetta_);    //Отправка углов на контроллер

        t_ = std::chrono::steady_clock::now();
        data_joints_ << 1, (double)(std::chrono::duration_cast<std::chrono::microseconds>(t_ - init_time_).count()), thetta_[0], thetta_[1], thetta_[2], thetta_[3], thetta_[4], thetta_[5], thetta_[6]; 
        log_.setData(data_joints_);

        // ---------------------------------------------------------------------------------------------------

        std::cout << "Статус: " << state_ << std::endl;
        std::cout << "Рассчитанные углы: " <<  thetta_.transpose()*180/M_PI << std::endl;
        
        // std::cout << "Рассчитанные углы: " <<  server::eigenArrayToJson(params.thetta_).dump().c_str() << std::endl;

        last_time_ = std::chrono::steady_clock::now();
        std::cout << "Время: " << std::chrono::duration_cast<std::chrono::microseconds>(last_time_ - time_).count() << std::endl << std::endl;

    }   // =============================================================================================================
    else if ((btn_2) && (std::chrono::duration_cast<std::chrono::microseconds>(time_ - last_time_).count() >= 25000)) // =============================
    {   // =============================================================================================================

        delta_position_ = position_ - previous_position_;       // Смещение хаптика отнисительно предыдущего положения

        temp_ = current_pos_;       // Сохранение предыдущего положения

        // Изменение положения на смещение 
        current_pos_.x() = current_pos_.x() - delta_position_[2]/trans_factor_*2;
        current_pos_.y() = current_pos_.y() - delta_position_[0]/trans_factor_*2;
        current_pos_.z() = current_pos_.z() + delta_position_[1]/trans_factor_*2;

        if (!checkPos())    // Проверка на выход за пределы разрешенной области 
        {   
            current_pos_ = temp_;   // Откат к предыдущиму положению
            std::cout << std::endl << "False" << std::endl;
        }

        std::cout << "Текущая позиция:\n" << current_pos_.x() << "\t" << current_pos_.y() << "\t" << current_pos_.z() << std::endl;
        
        previous_position_ = position_;     // Сохраннение предыдущей позиции (для расчета смещения)

        current_rot_ = hapticFK(joint_angles_, wrist_angles_);      // Расчет ориентации

        std::cout << std::endl << "Текущая матрица:\n" << current_rot_ << std::endl;

        t_ = std::chrono::steady_clock::now();;

        // ---------------------------------------------------------------------------------------------------

        t_ = std::chrono::steady_clock::now();
        data_time_[1] = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t_ - init_time_).count());
        log_.setData(data_time_);

        // Решение обратной кинематики
        kinematic_.setPositionVector(current_pos_);
        kinematic_.setRotationMatrix(current_rot_);
        state_ = kinematic_.IK();
        thetta_ = kinematic_.getQ();

        t_ = std::chrono::steady_clock::now();
        data_time_[1] = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t_ - init_time_).count());
        log_.setData(data_time_);

        // ---------------------------------------------------------------------------------------------------

        server_.setMsg(thetta_);    //Отправка углов на контроллер

        t_ = std::chrono::steady_clock::now();
        data_joints_ << 1, (double)(std::chrono::duration_cast<std::chrono::microseconds>(t_ - init_time_).count()), thetta_[0], thetta_[1], thetta_[2], thetta_[3], thetta_[4], thetta_[5], thetta_[6]; 
        log_.setData(data_joints_);

        // ---------------------------------------------------------------------------------------------------

        std::cout << "Статус: " << state_ << std::endl;
        std::cout << "Рассчитанные углы: " <<  thetta_.transpose()*180/M_PI << std::endl;
        
        // std::cout << "Рассчитанные углы: " <<  server::eigenArrayToJson(params.thetta_).dump().c_str() << std::endl;

        last_time_ = std::chrono::steady_clock::now();;
        std::cout << "Время: " << std::chrono::duration_cast<std::chrono::microseconds>(last_time_ - time_).count() << std::endl << std::endl;

    }   // =============================================================================================================
    else if (!(btn_1) && !(btn_2))   // ================================================================================
    {   // =============================================================================================================

        previous_position_ = position_;
        
    }
}

hduVector3Dd params::TeleState::getForceVector()
{
    if (server_.getMsg(torque_msg_))
    {
        // std::cout << params.torque_msg_.transpose() << std::endl;

        // Текущие углы куки
        current_kuka_thetta_ << torque_msg_[0], torque_msg_[1], torque_msg_[2], torque_msg_[3], torque_msg_[4], torque_msg_[5], torque_msg_[6]; 
        // Текущие торки в джоинтах куки
        current_kuka_torque_ << torque_msg_[7], torque_msg_[8], torque_msg_[9], torque_msg_[10], torque_msg_[11], torque_msg_[12], torque_msg_[13];
        
        // if (first_msg_)
        // {
        //     previous_kuka_d_thetta_ << 0,0,0,0,0,0,0;
        //     current_kuka_d_thetta_ << 0,0,0,0,0,0,0;
        //     current_kuka_dd_thetta_ << 0,0,0,0,0,0,0;
        //     previous_kuka_thetta_ = current_kuka_thetta_;
        //     first_msg_ = false;
        // }
        // else
        // {
        //     dt_ = (double)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - last_msg_).count())/1000000;
        //     current_kuka_d_thetta_ = (current_kuka_thetta_ - previous_kuka_thetta_)/dt_;
        //     current_kuka_dd_thetta_ = (current_kuka_d_thetta_ - previous_kuka_d_thetta_)/dt_;
        //     previous_kuka_thetta_ = current_kuka_thetta_;
        //     previous_kuka_d_thetta_ = current_kuka_d_thetta_;
        // }

        // inertia_kuka_torque_ = kinematic_.getTorque(current_kuka_thetta_, current_kuka_d_thetta_, current_kuka_dd_thetta_);

        // std::cout << "q: " << current_kuka_thetta_.transpose() << std::endl;
        // std::cout << "dq: " << current_kuka_d_thetta_.transpose() << std::endl;
        // std::cout << "ddq: " << current_kuka_dd_thetta_.transpose() << std::endl;
        
        // std::cout << "Current: " << current_kuka_torque_.transpose() << std::endl;
        // std::cout << "Inertia: " << inertia_kuka_torque_.transpose() << std::endl;

        // last_msg_ = std::chrono::steady_clock::now();

        // ---------------------------------------------------------------------------------------------------

        t_ = std::chrono::steady_clock::now();
        data_joints_ << 2, (double)(std::chrono::duration_cast<std::chrono::microseconds>(t_ - init_time_).count()), current_kuka_thetta_[0], current_kuka_thetta_[1], current_kuka_thetta_[2], current_kuka_thetta_[3], current_kuka_thetta_[4], current_kuka_thetta_[5], current_kuka_thetta_[6]; 
        log_.setData(data_joints_);

        // ---------------------------------------------------------------------------------------------------

        // Расчет силы на эндефекторе
        force_ = kinematic_.getForce(current_kuka_thetta_, current_kuka_torque_);

        // Масштабирование вектора силы
        forceVec_[2] = std::clamp(-force_[3]/force_factor_, -3., 3.);
        forceVec_[0] = std::clamp(-force_[4]/force_factor_, -3., 3.);
        forceVec_[1] = std::clamp( force_[5]/force_factor_, -3., 3.);

        // Задание силы
        // std::cout << "Force: " << forceVec[0] << "\t" << forceVec[1] << "\t" << forceVec[2] << std::endl;
    }

    return forceVec_;
}

void params::TeleState::setConnection()
{
    const char* name = "/my_shm1";

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

    munmap(ptr, sizeof(bool));
    close(shm_fd);
}

// ==================================================================================

bool params::TeleState::checkPos()
{
    if ((current_pos_ - initial_pos_).norm() <= radius_)
    {
        return true;
    }
    else
    {
        return false;
    }
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