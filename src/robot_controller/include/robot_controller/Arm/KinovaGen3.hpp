#pragma once
#include <robot_controller/Arm/ArmBase.hpp>

namespace k_api = Kinova::Api;
namespace rc
{
class KinovaGen3 : public ArmBase
{
public:
    KinovaGen3(const std::string& ip_addr, const uint32_t& port,
               const std::string& username, const std::string& passwd);
    ~KinovaGen3() override;
    void ExecuteTwistCommand(const Vector6f& twist_vel_cmd) override;
    void UpdateFeedback() override;
    void SetGripperPosition(float position) override;
    void KeepPosition();
    void Stop() override;

private:
    static constexpr int JOINT_COUNT = 7;
    void InitCommunication();
    const std::string ip_addr_, username_, passwd_;
    const uint32_t port_;
    std::shared_ptr<k_api::TransportClientTcp> transport_ptr_;
    std::shared_ptr<k_api::RouterClient> router_ptr_;
    std::shared_ptr<k_api::Session::CreateSessionInfo> session_info_ptr_;
    std::shared_ptr<k_api::SessionManager> session_manager_ptr_;
    std::shared_ptr<k_api::Base::BaseClient> base_ptr_;
    std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_ptr_;
    k_api::BaseCyclic::Feedback feed_back_;
};
}
