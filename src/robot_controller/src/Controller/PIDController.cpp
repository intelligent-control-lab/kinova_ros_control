#include <robot_controller/Controller/PIDController.hpp>
#include <spdlog/spdlog.h>

namespace rc
{
PIDController::PIDController(const std::string& name, const float& kp, const float& ki, const float& kd)
    : Kp_(kp), Ki_(ki), Kd_(kd)
{
    SPDLOG_INFO("PID {}: Kp={}, Ki={}, Kd={}", name, kp, ki, kd);
}

float PIDController::UpdateByError(const std::pair<float, float>& error_dt)
{
    const float error = error_dt.first;
    const float dt = error_dt.second;
    e_sum_ += error;
    if (first_u_)
    {
        e_last_ = error;
        first_u_ = false;
    }
    // Preserve the controller's discrete integral and first-derivative convention.
    const float output = Kp_ * error + Ki_ * e_sum_ * dt + Kd_ * (error - e_last_) / dt;
    e_last_ = error;
    return output;
}
}
