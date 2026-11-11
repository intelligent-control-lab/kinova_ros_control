#include <robot_controller/Arm/KinovaGen3.hpp>

namespace rc
{
    KinovaGen3::KinovaGen3(const std::string& ip_addr, const uint32_t& port,
                            const std::string& username, const std::string& passwd):
        ArmBase{JOINT_COUNT,
                ToEigen<float>(std::vector<float>{   0.0f,      0.0f,       0.0f,     0.0f,       0.0f,     0.0f,      0.0f,     0.0f }), // a
                ToEigen<float>(std::vector<float>{   M_PI,    M_PI_2,  1.5f*M_PI,   M_PI_2,  1.5f*M_PI,   M_PI_2, 1.5f*M_PI,     M_PI }), // alpha
                ToEigen<float>(std::vector<float>{ 0.1564f, -0.1284f,   -0.0118f, -0.4208f,   -0.0128f, -0.3143f,      0.0f, -0.1674f - 0.12f }) // d
            }, ip_addr_{ip_addr}, port_{port}, username_{username}, passwd_{passwd}
    {
        this->InitCommunication();

    }

    KinovaGen3::~KinovaGen3()
    {
        try { Stop(); }
        catch (const std::exception& e) { SPDLOG_ERROR("Shutdown failed: {}", e.what()); }
    }

    void KinovaGen3::InitCommunication()
    {
        try
        {
            SPDLOG_INFO("Init com...");
            transport_ptr_ = std::make_shared<k_api::TransportClientTcp>();
            router_ptr_ = std::make_shared<k_api::RouterClient>(&(*transport_ptr_),
                [](k_api::KError err){
                    cout << "_________ callback error _________" << err.toString();
                }
            );
            transport_ptr_->connect(ip_addr_, port_);

            session_info_ptr_ = std::make_shared<k_api::Session::CreateSessionInfo>();
            session_info_ptr_->set_username(username_);
            session_info_ptr_->set_password(passwd_);
            session_info_ptr_->set_session_inactivity_timeout(6e4); // ms
            session_info_ptr_->set_connection_inactivity_timeout(2e3);

            SPDLOG_INFO("Creating session for communication...");
            session_manager_ptr_ = std::make_shared<k_api::SessionManager>(&(*router_ptr_));
            session_manager_ptr_->CreateSession(*session_info_ptr_);

            base_ptr_ = std::make_shared<k_api::Base::BaseClient>(&(*router_ptr_));
            base_cyclic_ptr_ = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(&(*router_ptr_));

            if (base_ptr_->GetActuatorCount().count() != JOINT_COUNT)
                throw std::runtime_error("This controller requires a 7-DOF Kinova Gen3");

            SPDLOG_INFO("Kinova initialized.");
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }
    }

    void KinovaGen3::Stop()
    {
        try
        {
            if (session_manager_ptr_ != nullptr)
            {
                this->KeepPosition();

                session_manager_ptr_->CloseSession();
                router_ptr_->SetActivationStatus(false);
                transport_ptr_->disconnect();

                transport_ptr_ = nullptr;
                router_ptr_ = nullptr;
                session_manager_ptr_ = nullptr;
            }
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

    void KinovaGen3::KeepPosition()
    {
        try
        {
            k_api::Base::JointSpeeds joint_speeds_stop;
            std::vector<float> stop = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            for(size_t i = 0 ; i < stop.size(); ++i)
            {
                auto joint_speed = joint_speeds_stop.add_joint_speeds();
                joint_speed->set_joint_identifier(i);
                joint_speed->set_value(stop.at(i));
                joint_speed->set_duration(1);
            }
            base_ptr_->SendJointSpeedsCommand(joint_speeds_stop);
            SPDLOG_WARN("Enforcing current position...");
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

    void KinovaGen3::UpdateFeedback()
    {
        try
        {

            feed_back_ = base_cyclic_ptr_->RefreshFeedback();

            pEE_euler_(0) = feed_back_.base().tool_pose_x();
            pEE_euler_(1) = feed_back_.base().tool_pose_y();
            pEE_euler_(2) = feed_back_.base().tool_pose_z();

            pEE_euler_(5) = DEG2RAD(feed_back_.base().tool_pose_theta_z());
            pEE_euler_(4) = DEG2RAD(feed_back_.base().tool_pose_theta_y());
            pEE_euler_(3) = DEG2RAD(feed_back_.base().tool_pose_theta_x());

            // External wrench (force + torque)
            tool_external_wrench_(0) = feed_back_.base().tool_external_wrench_force_x();
            tool_external_wrench_(1) = feed_back_.base().tool_external_wrench_force_y();
            tool_external_wrench_(2) = feed_back_.base().tool_external_wrench_force_z();
            tool_external_wrench_(3) = feed_back_.base().tool_external_wrench_torque_x();
            tool_external_wrench_(4) = feed_back_.base().tool_external_wrench_torque_y();
            tool_external_wrench_(5) = feed_back_.base().tool_external_wrench_torque_z();

            if (feed_back_.interconnect().gripper_feedback().motor_size() > 0)
            {
                gripper_position_ = feed_back_.interconnect().gripper_feedback().motor(0).position() / 100.0f; // Convert to 0.0-1.0
                gripper_velocity_ = feed_back_.interconnect().gripper_feedback().motor(0).velocity();
            }

            assert(q_.size() == Nj_);
            for (size_t i = 0; i < Nj_; ++i)
            {
                q_(i) = DEG2RAD(feed_back_.actuators(i).position());
                q_dot_(i) = DEG2RAD(feed_back_.actuators(i).velocity());
                q_torque_(i) = feed_back_.actuators(i).torque();
            }

        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }

    }

    void KinovaGen3::SetGripperPosition(float position)
    {
        try
        {
            position = std::max(0.0f, std::min(1.0f, position));

            k_api::Base::GripperCommand gripper_command;
            gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);

            auto finger = gripper_command.mutable_gripper()->add_finger();
            finger->set_finger_identifier(1);
            finger->set_value(position);

            base_ptr_->SendGripperCommand(gripper_command);
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }
    }

    void KinovaGen3::ExecuteTwistCommand(const Vector6f& twist_vel_cmd)
    {
        try
        {
            auto command = k_api::Base::TwistCommand();
            command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
            command.set_duration(0);  // Unlimited time to execute

            auto twist = command.mutable_twist();
            twist->set_linear_x(twist_vel_cmd(0));
            twist->set_linear_y(twist_vel_cmd(1));
            twist->set_linear_z(twist_vel_cmd(2));
            twist->set_angular_x(twist_vel_cmd(3));
            twist->set_angular_y(twist_vel_cmd(4));
            twist->set_angular_z(twist_vel_cmd(5));
            base_ptr_->SendTwistCommand(command);
        }
        catch(const std::exception& e)
        {
            SPDLOG_ERROR("{}", e.what());
            throw;
        }
    }

}
