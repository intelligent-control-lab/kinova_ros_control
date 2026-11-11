#include <kinova/kinova_bridge.hpp>
#include <cmath>

KinovaBridge::KinovaBridge(ros::NodeHandle& nh, const std::string& ip_addr, const uint32_t& port,
                            const std::string& username, const std::string& passwd,
                            const std::vector<std::tuple<std::string, float, float, float>>& params,
                            const std::vector<std::tuple<std::string, float>>& vel_limits) :
    frame_id_{"kinova_base"},
    last_gripper_cmd_{0.0f}, gripper_cmd_initialized_{false}
{
    std::shared_ptr<rc::ArmBase> arm_ptr = std::make_shared<rc::KinovaGen3>(ip_addr, port, username, passwd);

    std::vector<std::tuple<std::string, float, float, float, float>> admittance_params;

    bool admittance_enabled = false;
    nh.param<bool>("admittance_control/enabled", admittance_enabled, false);

    if (admittance_enabled)
    {
        ROS_INFO("Loading admittance control parameters...");

        std::string x_name;
        float x_k, x_d, x_f_dead, x_max_offset;
        nh.param<std::string>("admittance_control/x/name", x_name, "x");
        nh.param<float>("admittance_control/x/k", x_k, 1000.0f);
        nh.param<float>("admittance_control/x/D", x_d, 100.0f);
        nh.param<float>("admittance_control/x/f_dead", x_f_dead, 10.0f);
        nh.param<float>("admittance_control/x/max_offset", x_max_offset, 0.005f);
        admittance_params.emplace_back(x_name, x_k, x_d, x_f_dead, x_max_offset);
        ROS_INFO("  X: K=%.1f N/m, D=%.1f Ns/m, f_dead=%.1f N, max_offset=%.4f m",
                 x_k, x_d, x_f_dead, x_max_offset);

        std::string y_name;
        float y_k, y_d, y_f_dead, y_max_offset;
        nh.param<std::string>("admittance_control/y/name", y_name, "y");
        nh.param<float>("admittance_control/y/k", y_k, 1000.0f);
        nh.param<float>("admittance_control/y/D", y_d, 100.0f);
        nh.param<float>("admittance_control/y/f_dead", y_f_dead, 10.0f);
        nh.param<float>("admittance_control/y/max_offset", y_max_offset, 0.005f);
        admittance_params.emplace_back(y_name, y_k, y_d, y_f_dead, y_max_offset);
        ROS_INFO("  Y: K=%.1f N/m, D=%.1f Ns/m, f_dead=%.1f N, max_offset=%.4f m",
                 y_k, y_d, y_f_dead, y_max_offset);

        std::string z_name;
        float z_k, z_d, z_f_dead, z_max_offset;
        nh.param<std::string>("admittance_control/z/name", z_name, "z");
        nh.param<float>("admittance_control/z/k", z_k, 1000.0f);
        nh.param<float>("admittance_control/z/D", z_d, 100.0f);
        nh.param<float>("admittance_control/z/f_dead", z_f_dead, 10.0f);
        nh.param<float>("admittance_control/z/max_offset", z_max_offset, 0.005f);
        admittance_params.emplace_back(z_name, z_k, z_d, z_f_dead, z_max_offset);
        ROS_INFO("  Z: K=%.1f N/m, D=%.1f Ns/m, f_dead=%.1f N, max_offset=%.4f m",
                 z_k, z_d, z_f_dead, z_max_offset);
    }
    else
    {
        ROS_INFO("Admittance control disabled, using default parameters");
        // Default parameters (will not be used unless enabled via EnableAdmittance())
        admittance_params.emplace_back("x", 1000.0f, 100.0f, 10.0f, 0.005f);
        admittance_params.emplace_back("y", 1000.0f, 100.0f, 10.0f, 0.005f);
        admittance_params.emplace_back("z", 1000.0f, 100.0f, 10.0f, 0.005f);
    }

    process_ptr_ = std::make_shared<rc::CartesianFollowingTwist>(arm_ptr, params, vel_limits, admittance_params);

    if (admittance_enabled)
    {
        process_ptr_->EnableAdmittance(true);
        ROS_INFO("Admittance control ENABLED");
    }

    wiggling_service_ = nh.advertiseService(
        "set_wiggling", &KinovaBridge::CallbackSetWiggling, this);
    ROS_INFO("Advertised set_wiggling service (std_srvs/SetBool)");

    pose_pub_            = nh.advertise<geometry_msgs::PoseStamped>("feedback_cartesian", 1000);
    pose_fk_pub_         = nh.advertise<geometry_msgs::PoseStamped>("feedback_cartesian_fk", 1000);
    joint_state_pub_     = nh.advertise<sensor_msgs::JointState>("feedback_joint_states", 1000);
    external_wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("feedback_external_wrench", 1000);
    gripper_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("feedback_gripper_state", 1000);

    admittance_compensated_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("admittance_compensated_pose", 1000);
    admittance_offset_pub_ = nh.advertise<geometry_msgs::Point>("admittance_offset", 1000);

    process_ptr_->UpdateFeedBack();

    // Use FK-computed pose from joint positions instead of direct feedback
    init_pose_ = Eigen::Affine3f(process_ptr_->arm_ptr_->pose_mat_fk());
    ROS_INFO("Initializing at current pose (FK):");
    ROS_INFO("  Position: [%.3f, %.3f, %.3f]", process_ptr_->x_fk(), process_ptr_->y_fk(), process_ptr_->z_fk());
    ROS_INFO("  Orientation: [%.3f, %.3f, %.3f] deg",
             RAD2DEG(process_ptr_->rx_fk()), RAD2DEG(process_ptr_->ry_fk()), RAD2DEG(process_ptr_->rz_fk()));

    pose_ref_buf_ = init_pose_;

    ROS_INFO("KinovaBridge initialized.");
}

KinovaBridge::~KinovaBridge()
{

}

void KinovaBridge::CallbackCartesian(const geometry_msgs::PoseStamped& pose)
{
    if (!pose.header.frame_id.empty() && pose.header.frame_id != frame_id_)
    {
        ROS_WARN_THROTTLE(1.0, "Ignoring Cartesian target outside kinova_base");
        return;
    }
    Eigen::Vector3f position{static_cast<float>(pose.pose.position.x),
                             static_cast<float>(pose.pose.position.y),
                             static_cast<float>(pose.pose.position.z)};
    Eigen::Quaternionf q{static_cast<float>(pose.pose.orientation.w), static_cast<float>(pose.pose.orientation.x),
                         static_cast<float>(pose.pose.orientation.y), static_cast<float>(pose.pose.orientation.z)};
    const float norm = q.norm();
    if (!position.allFinite() || !q.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-6f)
    {
        ROS_WARN_THROTTLE(1.0, "Ignoring non-finite Cartesian target or invalid quaternion");
        return;
    }
    q.normalize();
    Eigen::Affine3f pose_ref_recv{Eigen::Translation3f{position}};
    pose_ref_recv.rotate(q);

    std::unique_lock<std::mutex> lock{ref_mutex_};
    pose_ref_buf_ = pose_ref_recv;
    lock.unlock();
}

bool KinovaBridge::CallbackSetWiggling(std_srvs::SetBool::Request& request,
                                       std_srvs::SetBool::Response& response)
{
    process_ptr_->EnableWiggling(request.data);
    response.success = true;
    response.message = request.data ? "XY wiggling enabled" : "XY wiggling disabled";
    ROS_INFO_STREAM(response.message);
    return true;
}

void KinovaBridge::CallbackGripper(const sensor_msgs::JointState& gripper_cmd)
{
    if (gripper_cmd.position.size() > 0)
    {
        float desired_position = gripper_cmd.position[0];
        if (!std::isfinite(desired_position))
        {
            ROS_WARN_THROTTLE(1.0, "Ignoring non-finite gripper target");
            return;
        }

        desired_position = std::max(0.0f, std::min(1.0f, static_cast<float>(desired_position)));

        const float cmd_threshold = 0.05f;

        if (gripper_cmd_initialized_ && std::abs(desired_position - last_gripper_cmd_) < cmd_threshold)
        {
            ROS_DEBUG_THROTTLE(1.0, "Gripper command too close to previous (%.2f vs %.2f), skipping",
                              desired_position, last_gripper_cmd_);
            return;
        }

        ROS_INFO("Setting gripper to position: %.2f (0.0=open, 1.0=closed)", desired_position);

        process_ptr_->SetGripperPosition(desired_position);

        last_gripper_cmd_ = desired_position;
        gripper_cmd_initialized_ = true;
    }
    else
    {
        ROS_WARN("Received gripper command with empty position field");
    }
}

void KinovaBridge::RefreshCommand()
{
    std::unique_lock<std::mutex> lock_pose{ref_mutex_};
    Eigen::Affine3f pose_ref_send{pose_ref_buf_};
    lock_pose.unlock();
    process_ptr_->SetTargetCartesian(pose_ref_send);
    process_ptr_->Tick();
}

void KinovaBridge::Publish()
{
    try
    {

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = process_ptr_->x();
        pose.pose.position.y = process_ptr_->y();
        pose.pose.position.z = process_ptr_->z();

        Eigen::Quaternionf q = Eigen::AngleAxisf{process_ptr_->rz(), Eigen::Vector3f::UnitZ()}
                                *Eigen::AngleAxisf{process_ptr_->ry(), Eigen::Vector3f::UnitY()}
                                *Eigen::AngleAxisf{process_ptr_->rx(), Eigen::Vector3f::UnitX()};

        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        pose_pub_.publish(pose);

        pose.pose.position.x = process_ptr_->x_fk();
        pose.pose.position.y = process_ptr_->y_fk();
        pose.pose.position.z = process_ptr_->z_fk();

        Eigen::Quaternionf q_fk = Eigen::AngleAxisf{process_ptr_->rz_fk(), Eigen::Vector3f::UnitZ()}
                                *Eigen::AngleAxisf{process_ptr_->ry_fk(), Eigen::Vector3f::UnitY()}
                                *Eigen::AngleAxisf{process_ptr_->rx_fk(), Eigen::Vector3f::UnitX()};

        pose.pose.orientation.w = q_fk.w();
        pose.pose.orientation.x = q_fk.x();
        pose.pose.orientation.y = q_fk.y();
        pose.pose.orientation.z = q_fk.z();

        pose_fk_pub_.publish(pose);

        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.header.frame_id = frame_id_;

        Eigen::VectorXf joint_position_eigen = process_ptr_->arm_ptr_->q();
        Eigen::VectorXf joint_velocity_eigen = process_ptr_->arm_ptr_->q_dot();
        Eigen::VectorXf joint_torque_eigen = process_ptr_->arm_ptr_->q_torque();

        size_t num_joints = joint_position_eigen.size();

        joint_state_msg.name.resize(num_joints);
        for (size_t i=0; i<num_joints; ++i)
        {
            joint_state_msg.name[i] = "joint_" + std::to_string(i+1);
        }

        // Joint positions (in radians)
        joint_state_msg.position.resize(num_joints);
        for (size_t i=0; i<num_joints; ++i)
        {
            joint_state_msg.position[i] = joint_position_eigen(i);
        }

        // Joint velocities (in rad/s)
        joint_state_msg.velocity.resize(num_joints);
        for (size_t i=0; i<num_joints; ++i)
        {
            joint_state_msg.velocity[i] = joint_velocity_eigen(i);
        }

        // Joint torques (in Nm)
        joint_state_msg.effort.resize(num_joints);
        for (size_t i=0; i<num_joints; ++i)
        {
            joint_state_msg.effort[i] = joint_torque_eigen(i);
        }

        joint_state_pub_.publish(joint_state_msg);

        geometry_msgs::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.header.frame_id = frame_id_;

        const auto& wrench = process_ptr_->arm_ptr_->ToolExternalWrench();
        wrench_msg.wrench.force.x = wrench(0);
        wrench_msg.wrench.force.y = wrench(1);
        wrench_msg.wrench.force.z = wrench(2);
        wrench_msg.wrench.torque.x = wrench(3);
        wrench_msg.wrench.torque.y = wrench(4);
        wrench_msg.wrench.torque.z = wrench(5);

        external_wrench_pub_.publish(wrench_msg);

        sensor_msgs::JointState gripper_joint_state_msg;
        gripper_joint_state_msg.header.stamp = ros::Time::now();
        gripper_joint_state_msg.header.frame_id = frame_id_;

        gripper_joint_state_msg.name.resize(1);
        gripper_joint_state_msg.name[0] = "gripper_joint";

        // Gripper position (0.0 = open, 1.0 = closed)
        gripper_joint_state_msg.position.resize(1);
        gripper_joint_state_msg.position[0] = process_ptr_->arm_ptr_->GripperPosition();

        gripper_joint_state_msg.velocity.resize(1);
        gripper_joint_state_msg.velocity[0] = process_ptr_->arm_ptr_->GripperVelocity();

        // No effort data available for gripper
        gripper_joint_state_msg.effort.resize(1);
        gripper_joint_state_msg.effort[0] = 0.0;

        gripper_joint_state_pub_.publish(gripper_joint_state_msg);

        auto cart_twist_ptr = process_ptr_;
        if (cart_twist_ptr->IsAdmittanceEnabled())
        {
            const Eigen::Affine3f& comp_pose = cart_twist_ptr->GetCompensatedPose();
            geometry_msgs::PoseStamped admittance_compensated_pose_msg;
            admittance_compensated_pose_msg.header.stamp = ros::Time::now();
            admittance_compensated_pose_msg.header.frame_id = frame_id_;

            admittance_compensated_pose_msg.pose.position.x = comp_pose.translation().x();
            admittance_compensated_pose_msg.pose.position.y = comp_pose.translation().y();
            admittance_compensated_pose_msg.pose.position.z = comp_pose.translation().z();

            Eigen::Quaternionf q_comp(comp_pose.rotation());
            admittance_compensated_pose_msg.pose.orientation.w = q_comp.w();
            admittance_compensated_pose_msg.pose.orientation.x = q_comp.x();
            admittance_compensated_pose_msg.pose.orientation.y = q_comp.y();
            admittance_compensated_pose_msg.pose.orientation.z = q_comp.z();

            admittance_compensated_pose_pub_.publish(admittance_compensated_pose_msg);

            const auto& offset = process_ptr_->GetAdmittanceOffset();
            geometry_msgs::Point offset_msg;
            offset_msg.x = offset.x();
            offset_msg.y = offset.y();
            offset_msg.z = offset.z();
            admittance_offset_pub_.publish(offset_msg);

        }

    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
        throw;
    }

}
