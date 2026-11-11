#pragma once

#include <robot_controller/Arm/ArmBase.hpp>
#include <robot_controller/Controller/PIDController.hpp>
#include "robot_controller/Controller/AdmittanceController.hpp"

namespace rc
{

    class CartesianFollowingTwist {
        public:
            explicit CartesianFollowingTwist(
                    std::shared_ptr<ArmBase> arm_ptr,
                    const std::vector<std::tuple<std::string, float, float, float>>& pid_params,
                    const std::vector<std::tuple<std::string, float>>& vel_limits = {
                        {"x", 0.2f}, {"y", 0.2f}, {"z", 0.2f},
                        {"rx", 20.0f}, {"ry", 20.0f}, {"rz", 20.0f}
                    },
                    const std::vector<std::tuple<std::string, float, float, float, float>>& admittance_params = {
                        {"x", 1000.0f, 100.0f, 1.0f, 0.05f},  // name, K, D, force_deadband, max_offset
                        {"y", 1000.0f, 100.0f, 1.0f, 0.05f},
                        {"z", 1000.0f, 100.0f, 1.0f, 0.05f}
                    });

            Eigen::MatrixXf Tick();
            void SetTargetCartesian(const Eigen::Affine3f& pose);

            void EnableAdmittance(const bool& enable)
            {
                enable_admittance_ = enable;
                if (!enable)
                {
                    for (auto& adm : admittance_ptr_)
                    {
                        adm.Reset();
                    }
                }
                SPDLOG_WARN("Admittance control: {}", enable_admittance_ ? "ENABLED" : "DISABLED");
            }

            bool IsAdmittanceEnabled() const { return enable_admittance_; }

            const Eigen::Affine3f& GetCompensatedPose() const { return admittance_compensated_pose_; }
            const Eigen::Vector3f& GetAdmittanceOffset() const { return admittance_offset_; }

            void EnableWiggling(const bool& enable)
            {
                enable_wiggling_ = enable;
                if (!enable)
                {
                    wiggle_active_ = false;
                    wiggle_time_ = 0.0f;
                }
                SPDLOG_WARN("Wiggling control: {}", enable_wiggling_ ? "ENABLED" : "DISABLED");
            }

            bool IsWigglingEnabled() const { return enable_wiggling_; }

            void UpdateFeedBack() { arm_ptr_->UpdateAll(); }
            void SetGripperPosition(float position) { arm_ptr_->SetGripperPosition(position); }
            float x() const { return arm_ptr_->x(); }
            float y() const { return arm_ptr_->y(); }
            float z() const { return arm_ptr_->z(); }
            float rx() const { return arm_ptr_->rx(); }
            float ry() const { return arm_ptr_->ry(); }
            float rz() const { return arm_ptr_->rz(); }
            float x_fk() const { return arm_ptr_->x_fk(); }
            float y_fk() const { return arm_ptr_->y_fk(); }
            float z_fk() const { return arm_ptr_->z_fk(); }
            float rx_fk() const { return arm_ptr_->rx_fk(); }
            float ry_fk() const { return arm_ptr_->ry_fk(); }
            float rz_fk() const { return arm_ptr_->rz_fk(); }

            std::shared_ptr<ArmBase> arm_ptr_;
            unsigned long long last_cmd_tick_us_;

        private:

            static constexpr int num_axes_ = 6;
            std::vector<PIDController> pids_ptr_;
            Eigen::Affine3f target_pose_;
            Vector6f vel_limits_; // velocity limits for [x, y, z, rx, ry, rz]

            std::vector<AdmittanceController> admittance_ptr_; // One controller per XYZ axis
            bool enable_admittance_;
            Eigen::Vector3f admittance_offset_ = Eigen::Vector3f::Zero();
            Eigen::Affine3f admittance_compensated_pose_; // The actual target pose with admittance compensation

            bool enable_wiggling_;           // Master enable/disable for wiggling
            float wiggle_time_;              // Accumulated time for wiggle motion (seconds)
            float wiggle_freq_;              // Wiggle frequency (Hz)
            float wiggle_amp_m_;             // Wiggle amplitude (meters)
            float fz_wiggle_threshold_;      // Force threshold to trigger wiggling (N)
            bool wiggle_active_;             // Whether wiggling is currently active

            Vector6f GetTwistErrorBaseFrame(const Eigen::Affine3f& target_pose);

    };
}
