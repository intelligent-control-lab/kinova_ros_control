#include <robot_controller/Controller/AdmittanceController.hpp>
#include <algorithm>
#include <cmath>

namespace rc
{
    AdmittanceController::AdmittanceController(
        const std::string& name,
        const float& K,
        const float& D,
        const float& force_deadband,
        const float& max_offset)
        : name_(name)
        , K_(K)
        , D_(D)
        , force_deadband_(force_deadband)
        , max_offset_(max_offset)
        , admittance_offset_(0.0f)
        , effective_force_(0.0f)
    {
        try
        {
            SPDLOG_INFO("AdmittanceController [{}] initialized - K: {:.3f}, D: {:.3f}, Deadband: {:.3f}, Max Offset: {:.3f}",
                        name_, K_, D_, force_deadband_, max_offset_);
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("AdmittanceController initialization error: {}", e.what());
            throw;
        }
    }

    AdmittanceController::~AdmittanceController()
    {
    }

    void AdmittanceController::Reset()
    {
        admittance_offset_ = 0.0f;
        effective_force_ = 0.0f;
        SPDLOG_DEBUG("AdmittanceController [{}] reset", name_);
    }

    float AdmittanceController::ApplyDeadband(const float& force)
    {
        if (std::abs(force) < force_deadband_)
        {
            return 0.0f;
        }
        else if (force > 0)
        {
            return force - force_deadband_;
        }
        else
        {
            return force + force_deadband_;
        }
    }

    float AdmittanceController::SaturateOffset(const float& offset)
    {
        return std::max(-max_offset_, std::min(max_offset_, offset));
    }

    float AdmittanceController::Update(const float& force, const float& dt)
    {
        try
        {
            effective_force_ = ApplyDeadband(force);

            // First-order dynamics: D * dx/dt = f - K * x
            // Euler integration: x(t+dt) = x(t) + (f - K*x) / D * dt

            if (D_ < 1e-6f)
            {
                SPDLOG_WARN("AdmittanceController [{}]: Damping coefficient D is too small, using minimum value", name_);
                return admittance_offset_;
            }

            float velocity = (effective_force_ - K_ * admittance_offset_) / D_;

            admittance_offset_ += velocity * dt;

            admittance_offset_ = SaturateOffset(admittance_offset_);

            return admittance_offset_;
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("AdmittanceController [{}] Update error: {}", name_, e.what());
            throw;
        }
    }
}
