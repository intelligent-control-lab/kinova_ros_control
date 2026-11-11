#include <robot_controller/Arm/ArmBase.hpp>

namespace rc
{
    ArmBase::ArmBase(int n_joint, Eigen::VectorXf a, Eigen::VectorXf alpha, Eigen::VectorXf d) :
        Nj_{n_joint}, a_{a}, alpha_{alpha}, d_{d}
    {
        this->Init();
    }

    void ArmBase::Init()
    {
        try
        {
            q_ = Eigen::VectorXf::Zero(Nj_, 1);
            q_dot_ = q_;
            q_torque_ = q_;

            this->GenerateLinkTransform();

            pEE_euler_    = Eigen::VectorXf::Zero(6, 1);
            pEE_euler_fk_ = Eigen::VectorXf::Zero(6, 1);
            tool_external_wrench_ = Eigen::VectorXf::Zero(6, 1);

            gripper_position_ = 0.0f;
            gripper_velocity_ = 0.0f;

        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

    void ArmBase::GenerateLinkTransform()
    {
        try
        {
            link_transform_.clear();
            for (size_t i = 0; i < Nj_ + 1; ++i)
            {
                link_transform_.emplace_back(
                    [this, i] (double q) {
                        Eigen::Matrix4f trans_mat;
                        trans_mat << cos(q), -sin(q)*cos(alpha_(i)),  sin(q)*sin(alpha_(i)), a_(i)*cos(q),
                                     sin(q),  cos(q)*cos(alpha_(i)), -cos(q)*sin(alpha_(i)), a_(i)*sin(q),
                                          0,         sin(alpha_(i)),         cos(alpha_(i)),        d_(i),
                                          0,                      0,                      0,            1;
                        return trans_mat;
                    }
                );
            }
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

    void ArmBase::UpdateDynamicState()
    {
        try
        {
            auto f_transform = link_transform_.at(0);
            auto trans_mat = f_transform(0.0f);

            for (size_t i = 0; i < Nj_; ++i)
            {
                f_transform = link_transform_.at(i + 1);
                trans_mat = trans_mat * f_transform(q_(i));
            }

            poseEE_mat_fk_ = trans_mat;

            PoseMatToTransEulerRad(poseEE_mat_fk_, pEE_euler_fk_);

        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

    void ArmBase::UpdateAll()
    {
        try
        {
            this->UpdateFeedback();
            this->UpdateDynamicState();
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

}
