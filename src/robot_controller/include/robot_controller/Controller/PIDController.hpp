#pragma once
#include <string>
#include <utility>

namespace rc
{
class PIDController
{
public:
    PIDController(const std::string& name, const float& kp, const float& ki, const float& kd);
    float UpdateByError(const std::pair<float, float>& error_dt);
    float e() const { return e_last_; }

private:
    const float Kp_, Ki_, Kd_;
    float e_last_ = 0.0f;
    float e_sum_ = 0.0f;
    bool first_u_ = true;
};
}
