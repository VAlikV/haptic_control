#include "tele_params/tele_params.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <poll.h>
#include <thread>
#include <unistd.h>

namespace
{
std::atomic_bool stop_requested{false};

struct CallbackContext
{
    params::TeleState& teleoperation;
    params::HapicState haptic_state{};
};

void handleSignal(int)
{
    stop_requested.store(true, std::memory_order_relaxed);
}

bool inputAvailable()
{
    pollfd input{};
    input.fd = STDIN_FILENO;
    input.events = POLLIN;
    return poll(&input, 1, 0) > 0 && (input.revents & POLLIN) != 0;
}

HDCallbackCode HDCALLBACK hapticCallback(void* data)
{
    auto& context = *static_cast<CallbackContext*>(data);
    const HHD device = hdGetCurrentDevice();

    hdBeginFrame(device);
    hdGetDoublev(HD_CURRENT_POSITION, context.haptic_state.position);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, context.haptic_state.joint_angles);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, context.haptic_state.wrist_angles);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &context.haptic_state.buttons);

    context.teleoperation.setHapticState(context.haptic_state);
    context.haptic_state.force = context.teleoperation.getForceVector();
    hdSetDoublev(HD_CURRENT_FORCE, context.haptic_state.force);
    hdEndFrame(device);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Ошибка в callback планировщика");
        if (hduIsSchedulerError(&error))
        {
            stop_requested.store(true, std::memory_order_relaxed);
            return HD_CALLBACK_DONE;
        }
    }

    return stop_requested.load(std::memory_order_relaxed)
        ? HD_CALLBACK_DONE
        : HD_CALLBACK_CONTINUE;
}
} // namespace

int main()
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    HDErrorInfo error;
    const HHD device = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Не удалось инициализировать haptic-устройство");
        return 1;
    }

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Не удалось запустить планировщик");
        hdDisableDevice(device);
        return 1;
    }

    params::TeleState teleoperation{0};
    teleoperation.setConnection();
    CallbackContext context{teleoperation};

    const HDSchedulerHandle callback = hdScheduleAsynchronous(
        hapticCallback, &context, HD_DEFAULT_SCHEDULER_PRIORITY);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Не удалось зарегистрировать callback");
        hdStopScheduler();
        hdDisableDevice(device);
        return 1;
    }

    std::puts("Телеуправление запущено. Нажмите Enter или Ctrl+C для выхода.");

    while (!stop_requested.load(std::memory_order_relaxed) &&
           hdWaitForCompletion(callback, HD_WAIT_CHECK_STATUS))
    {
        if (inputAvailable())
        {
            stop_requested.store(true, std::memory_order_relaxed);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    hdStopScheduler();
    hdUnschedule(callback);
    hdDisableDevice(device);
    return 0;
}
