#pragma once

#include <string>
#include <spdlog/spdlog.h>

namespace rc
{

    class AdmittanceController
    {
    public:

        AdmittanceController(
            const std::string& name,
            const float& K,
            const float& D,
            const float& force_deadband,
            const float& max_offset);

        ~AdmittanceController();

        float Update(const float& force, const float& dt);

        void Reset();

        float offset() const { return admittance_offset_; }

        float effective_force() const { return effective_force_; }

    private:
        std::string name_;

        const float K_;              // Stiffness (N/m)
        const float D_;              // Damping (N*s/m)
        const float force_deadband_; // Force deadband (N)
        const float max_offset_;     // Maximum offset (m)

        float admittance_offset_;    // Current displacement offset (m)
        float effective_force_;      // Force after applying deadband (N)

        float ApplyDeadband(const float& force);

        float SaturateOffset(const float& offset);
    };
}
