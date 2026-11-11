#pragma once

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <robot_controller/Arm/KinovaGen3.hpp>
#include <robot_controller/Process/CartesianFollowingTwist.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

class KinovaBridge {

    public:

        KinovaBridge(ros::NodeHandle& nh, const std::string& ip_addr, const uint32_t& port,
                            const std::string& username, const std::string& passwd,
                            const std::vector<std::tuple<std::string, float, float, float>>& params,
                            const std::vector<std::tuple<std::string, float>>& vel_limits = {
                                {"x", 0.2f}, {"y", 0.2f}, {"z", 0.2f},
                                {"rx", 20.0f}, {"ry", 20.0f}, {"rz", 20.0f}
                            });
        ~KinovaBridge();

        void Publish();
        void CallbackCartesian(const geometry_msgs::PoseStamped& pose);
        void RefreshCommand();
        void CallbackGripper(const sensor_msgs::JointState& gripper_cmd);
        bool CallbackSetWiggling(std_srvs::SetBool::Request& request,
                                 std_srvs::SetBool::Response& response);

    private:

        std::shared_ptr<rc::CartesianFollowingTwist> process_ptr_;

        const string frame_id_;

        ros::Publisher pose_pub_;
        ros::Publisher pose_fk_pub_;
        ros::Publisher joint_state_pub_;
        ros::Publisher external_wrench_pub_;
        ros::Publisher gripper_joint_state_pub_;
        ros::Publisher admittance_compensated_pose_pub_;
        ros::Publisher admittance_offset_pub_;
        ros::ServiceServer wiggling_service_;

        Eigen::Affine3f init_pose_;
        Eigen::Affine3f pose_ref_buf_;

        std::mutex ref_mutex_;

        float last_gripper_cmd_;
        bool gripper_cmd_initialized_;

};
