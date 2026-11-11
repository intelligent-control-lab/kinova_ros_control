#pragma once

#include <robot_controller/common.hpp>
#include <robot_controller/Util/Util.hpp>

namespace rc
{
    class ArmBase {
        public:

            explicit ArmBase(int n_joint, Eigen::VectorXf a, Eigen::VectorXf alpha, Eigen::VectorXf d);
            virtual ~ArmBase() = default;

            const float& x() const { return pEE_euler_(0); }
            const float& y() const { return pEE_euler_(1); }
            const float& z() const { return pEE_euler_(2); }
            const float& rx() const { return pEE_euler_(3); }
            const float& ry() const { return pEE_euler_(4); }
            const float& rz() const { return pEE_euler_(5); }

            const float& x_fk() const { return pEE_euler_fk_(0); }
            const float& y_fk() const { return pEE_euler_fk_(1); }
            const float& z_fk() const { return pEE_euler_fk_(2); }
            const float& rx_fk() const { return pEE_euler_fk_(3); }
            const float& ry_fk() const { return pEE_euler_fk_(4); }
            const float& rz_fk() const { return pEE_euler_fk_(5); }

            // Joint angles/velocities (rad)
            const Eigen::VectorXf& q() const { return q_; }
            const Eigen::VectorXf& q_dot() const { return q_dot_; }
            const Eigen::VectorXf& q_torque() const { return q_torque_; }

            const Eigen::Matrix4f& pose_mat_fk() const { return poseEE_mat_fk_; }

            // External wrench (force + torque)
            const Vector6f& ToolExternalWrench() const { return tool_external_wrench_; }

            float GripperPosition() const { return gripper_position_; }
            float GripperVelocity() const { return gripper_velocity_; }

            void UpdateAll();

            virtual void ExecuteTwistCommand(const Vector6f& twist_vel_cmd) = 0;
            virtual void UpdateFeedback() = 0;
            virtual void SetGripperPosition(float position) = 0;
            virtual void Stop() = 0;

        protected:

            const int Nj_; // number of joints
            Eigen::VectorXf q_, q_dot_; // joint angles (rad, rad/s)
            Eigen::VectorXf q_torque_; // joint torques (Nm)

            std::vector<std::function<Eigen::Matrix4f(float q)>> link_transform_; // Base->1, 1->2, ... , N->Tool
            const Eigen::VectorXf a_, alpha_, d_; // DH parameters. [0] base link, [1~N] joint link, m, rad

            Vector6f pEE_euler_; // Translation + Euler, Direct read

            Eigen::Matrix4f poseEE_mat_fk_; // tool pose in base frame
            Vector6f pEE_euler_fk_; // Translation + Euler, From poseEE_mat_fk_

            Vector6f tool_external_wrench_; // External wrench: [Fx, Fy, Fz, Tx, Ty, Tz]

            float gripper_position_; // Gripper position (0.0 - 1.0, 0=open, 1=closed)
            float gripper_velocity_; // Gripper velocity

        protected:

            void UpdateDynamicState();

        private:

            void Init();
            void GenerateLinkTransform();
    };
}
