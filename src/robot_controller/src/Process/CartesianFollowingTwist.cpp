#include <robot_controller/Process/CartesianFollowingTwist.hpp>

namespace rc
{
    // cartesian_params.at(i): name, kp, ki, kd
    // vel_limits.at(i): name, limit
    // admittance_params.at(i): name, K, D, force_deadband, max_offset
    CartesianFollowingTwist::CartesianFollowingTwist(
        std::shared_ptr<ArmBase> arm_ptr,
        const std::vector<std::tuple<std::string, float, float, float>>& cartesian_params,
        const std::vector<std::tuple<std::string, float>>& vel_limits,
        const std::vector<std::tuple<std::string, float, float, float, float>>& admittance_params):
        arm_ptr_(arm_ptr), last_cmd_tick_us_(GetTickUs()),
        enable_admittance_(false),
        enable_wiggling_(true),
        wiggle_time_(0.0f),
        wiggle_freq_(3.0f),           // Default: 3 Hz
        wiggle_amp_m_(0.001f),       // Default: 1 mm
        fz_wiggle_threshold_(10.0f),   // Default: 10 N
        wiggle_active_(false)
    {
        try
        {
            if (cartesian_params.size() != 6 || vel_limits.size() != 6 || admittance_params.size() != 3)
                throw std::invalid_argument("Expected six Cartesian PID/velocity entries and three admittance entries");
            for (const auto& p : cartesian_params)
                pids_ptr_.emplace_back(std::get<0>(p), std::get<1>(p), std::get<2>(p), std::get<3>(p));

            for (size_t i = 0; i < vel_limits.size() && i < 6; ++i)
            {
                vel_limits_(i) = std::get<1>(vel_limits[i]);
            }

            SPDLOG_INFO("CartesianFollowingTwist initialized with velocity limits:");
            for (size_t i = 0; i < vel_limits.size() && i < 6; ++i)
            {
                SPDLOG_INFO("  {}: {:.3f}", std::get<0>(vel_limits[i]), vel_limits_(i));
            }

            SPDLOG_INFO("Initializing admittance controllers:");
            for (size_t i = 0; i < admittance_params.size() && i < 3; ++i)
            {
                const auto& params = admittance_params[i];
                std::string name = std::get<0>(params);
                float K = std::get<1>(params);
                float D = std::get<2>(params);
                float force_deadband = std::get<3>(params);
                float max_offset = std::get<4>(params);

                admittance_ptr_.emplace_back(name, K, D, force_deadband, max_offset);
            }

            admittance_compensated_pose_ = Eigen::Affine3f::Identity();
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

    void CartesianFollowingTwist::SetTargetCartesian(const Eigen::Affine3f& pose)
    {
        try
        {
            target_pose_ = pose;
            admittance_compensated_pose_ = pose;
            if (enable_admittance_)
                admittance_compensated_pose_.translation() += admittance_offset_;

        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }
    }

    Vector6f CartesianFollowingTwist::GetTwistErrorBaseFrame(const Eigen::Affine3f& target_pose)
    {
        try
        {
            Eigen::Matrix3f R_base = target_pose.matrix().block<3, 3>(0, 0) *
                                arm_ptr_->pose_mat_fk().block<3, 3>(0, 0).inverse();
            Eigen::Vector3f T_base = target_pose.matrix().block<3, 1>(0, 3) -
                            arm_ptr_->pose_mat_fk().block<3, 1>(0, 3);
            Eigen::Matrix4f err_mat;
            err_mat << R_base, T_base,
                        0, 0, 0, 1;
            Vector6f err;
            PoseMatToTransAngRad(err_mat, err);

            return err;
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }
    }

    Eigen::MatrixXf CartesianFollowingTwist::Tick()
    {
        try
        {

            this->UpdateFeedBack();
            if (!arm_ptr_->pose_mat_fk().allFinite() || !arm_ptr_->ToolExternalWrench().allFinite())
                throw std::runtime_error("Non-finite robot feedback");
            auto now = GetTickUs();

            if (now - last_cmd_tick_us_ > 1000) // Max 1Khz
            {

                auto dt = (now - last_cmd_tick_us_)/1000000.0f; // seconds
                dt = min(dt, 0.1f);

                if (enable_admittance_)
                {
                    const Vector6f& base_wrench = arm_ptr_->ToolExternalWrench();

                    Eigen::Vector3f force_base = base_wrench.head<3>();

                    Eigen::Vector3f offset_xyz;
                    for (size_t i = 0; i < 3 && i < admittance_ptr_.size(); ++i)
                    {
                        offset_xyz(i) = admittance_ptr_[i].Update(force_base(i), dt);
                    }

                    admittance_offset_ = offset_xyz;

                    if (offset_xyz.norm() > 1e-3f)
                    {
                        SPDLOG_INFO("Admittance: Force=[{:.2f}, {:.2f}, {:.2f}] N, Offset=[{:.2f}, {:.2f}, {:.2f}] mm",
                                    force_base(0), force_base(1), force_base(2),
                                    offset_xyz(0) * 1000.0f, offset_xyz(1) * 1000.0f, offset_xyz(2) * 1000.0f);
                    }

                    admittance_compensated_pose_.translation() = target_pose_.translation() + offset_xyz;

                    admittance_compensated_pose_.linear() = target_pose_.linear();
                }
                else
                {
                    admittance_compensated_pose_ = target_pose_;
                    admittance_offset_ = Eigen::Vector3f::Zero();
                }

                Eigen::Affine3f final_target_pose = admittance_compensated_pose_;

                if (enable_wiggling_)
                {
                    const Vector6f& base_wrench = arm_ptr_->ToolExternalWrench();
                    float fz = base_wrench(2);  // Z-axis force component

                    if (std::abs(fz) > fz_wiggle_threshold_)
                    {
                        if (!wiggle_active_)
                        {
                            wiggle_active_ = true;
                            wiggle_time_ = 0.0f;  // Reset wiggle time when starting
                            SPDLOG_INFO("Wiggling ACTIVATED: Fz={:.2f} N exceeds threshold {:.2f} N",
                                        fz, fz_wiggle_threshold_);
                        }

                        wiggle_time_ += dt;

                        float omega = 2.0f * M_PI * wiggle_freq_;
                        float offset_x = wiggle_amp_m_ * std::cos(omega * wiggle_time_);
                        float offset_y = wiggle_amp_m_ * std::sin(omega * wiggle_time_);

                        Eigen::Vector3f wiggle_offset(offset_x, offset_y, 0.0f);
                        final_target_pose.translation() = admittance_compensated_pose_.translation() + wiggle_offset;

                        static float last_log_time = 0.0f;
                        if (wiggle_time_ - last_log_time > 1.0f)
                        {
                            SPDLOG_INFO("Wiggling: Fz={:.2f} N, Offset=[{:.2f}, {:.2f}] mm, Time={:.2f} s",
                                        fz, offset_x * 1000.0f, offset_y * 1000.0f, wiggle_time_);
                            last_log_time = wiggle_time_;
                        }
                    }
                    else
                    {
                        if (wiggle_active_)
                        {
                            wiggle_active_ = false;
                            wiggle_time_ = 0.0f;
                            SPDLOG_INFO("Wiggling DEACTIVATED: Fz={:.2f} N below threshold", fz);
                        }
                        final_target_pose = admittance_compensated_pose_;
                    }
                }
                else
                {
                    if (wiggle_active_)
                    {
                        wiggle_active_ = false;
                        wiggle_time_ = 0.0f;
                        SPDLOG_INFO("Wiggling stopped (disabled by user)");
                    }
                    final_target_pose = admittance_compensated_pose_;
                }

                Eigen::VectorXf U_twist(num_axes_);
                auto err = this->GetTwistErrorBaseFrame(final_target_pose);
                for (size_t i = 0; i < num_axes_; ++i)
                {
                    U_twist(i) = pids_ptr_.at(i).UpdateByError(std::make_pair(err(i), dt));
                }

                // convert rad/s to deg/s (only for rotation components)
                for (size_t i = 3; i < 6; ++i)
                {
                    U_twist(i) = RAD2DEG(U_twist(i));
                }

                for (size_t i = 0; i < 3; ++i)
                {
                    if (U_twist(i) > vel_limits_(i))
                    {
                        U_twist(i) = vel_limits_(i);
                    }

                    if (U_twist(i) < -vel_limits_(i))
                    {
                        U_twist(i) = -vel_limits_(i);
                    }
                }
                for (size_t i = 3; i < 6; ++i)
                {
                    if (U_twist(i) > vel_limits_(i))
                    {
                        U_twist(i) = vel_limits_(i);
                    }

                    if (U_twist(i) < -vel_limits_(i))
                    {
                        U_twist(i) = -vel_limits_(i);
                    }
                }

                if (!U_twist.allFinite())
                    throw std::runtime_error("Controller produced non-finite twist");
                arm_ptr_->ExecuteTwistCommand(U_twist);

                last_cmd_tick_us_ = now;
            }

            Vector6f error;
            for (size_t i = 0; i < num_axes_; ++i)
            {
                error(i) = pids_ptr_.at(i).e();
            }

            return error;

        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            try { arm_ptr_->Stop(); }
            catch (const std::exception& stop_error)
            {
                SPDLOG_ERROR("Failed to stop arm: {}", stop_error.what());
            }
            throw;
        }

    }

}
