#include <kinova/kinova_bridge.hpp>
#include <chrono>

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "kinova_bringup");
        ros::NodeHandle nh("~");

        std::string ip_addr, username, passwd;
        int port;
        nh.param<std::string>("ip_address", ip_addr, "192.168.1.10");
        nh.param<std::string>("username", username, "admin");
        nh.param<std::string>("password", passwd, "admin");
        nh.param<int>("port", port, 10000);
        if (port < 1 || port > 65535 || ip_addr.empty())
        {
            ROS_FATAL("Invalid connection parameters");
            return 1;
        }

        std::vector<std::tuple<std::string, float, float, float>> params;

        std::vector<std::tuple<std::string, float>> vel_limits = {
            {"x", 0.2f}, {"y", 0.2f}, {"z", 0.2f},
            {"rx", 20.0f}, {"ry", 20.0f}, {"rz", 20.0f}
        }; // default values

        std::vector<std::string> cart_names = {"x", "y", "z", "rx", "ry", "rz"};
        bool params_loaded = true;

        for (const auto& cart_name : cart_names)
        {
            std::string base_path = "cartesian_pid/" + cart_name + "/";
            std::string name;
            double kp, ki, kd;

            if (nh.getParam(base_path + "name", name) &&
                nh.getParam(base_path + "kp", kp) &&
                nh.getParam(base_path + "ki", ki) &&
                nh.getParam(base_path + "kd", kd))
            {
                params.push_back(std::make_tuple(name,
                    static_cast<float>(kp),
                    static_cast<float>(ki),
                    static_cast<float>(kd)));
            }
            else
            {
                params_loaded = false;
                break;
            }
        }

        if (!params_loaded)
        {
            ROS_WARN("Cartesian PID parameters not found in parameter server, using default values");
            params = {
                std::make_tuple("x ", 0.1f, 0.0f, 0.01f),
                std::make_tuple("y ", 0.1f, 0.0f, 0.01f),
                std::make_tuple("z ", 0.1f, 0.0f, 0.01f),
                std::make_tuple("rx", 2.0f, 0.0f, 0.02f),
                std::make_tuple("ry", 2.0f, 0.0f, 0.02f),
                std::make_tuple("rz", 2.0f, 0.0f, 0.02f)
            };
        }
        else
        {
            ROS_INFO("Cartesian PID parameters loaded from parameter server");
        }

        std::vector<std::string> vel_limit_names = {"x", "y", "z", "rx", "ry", "rz"};
        bool vel_limits_loaded = true;
        vel_limits.clear();

        for (const auto& vel_name : vel_limit_names)
        {
            std::string param_path = "vel_limits/" + vel_name;
            double limit;

            if (nh.getParam(param_path, limit))
            {
                vel_limits.push_back(std::make_tuple(vel_name, static_cast<float>(limit)));
            }
            else
            {
                vel_limits_loaded = false;
                break;
            }
        }

        if (!vel_limits_loaded)
        {
            vel_limits = {
                {"x", 0.2f}, {"y", 0.2f}, {"z", 0.2f},
                {"rx", 20.0f}, {"ry", 20.0f}, {"rz", 20.0f}
            };
            ROS_INFO("Using default velocity limits");
        }
        else
        {
            ROS_INFO("Velocity limits loaded from parameter server:");
            for (const auto& limit : vel_limits)
            {
                ROS_INFO("  %s: %.3f", std::get<0>(limit).c_str(), std::get<1>(limit));
            }
        }

        auto kinova_bridge_ptr = std::make_shared<KinovaBridge>(
            nh, ip_addr, port, username, passwd, params, vel_limits
        );

        auto ref_sub_cartesian = nh.subscribe("cartesian_target", 1000, &KinovaBridge::CallbackCartesian, &(*kinova_bridge_ptr));
        ROS_INFO("Subscribed cartesian_target");

        auto gripper_sub = nh.subscribe("gripper_target", 1000, &KinovaBridge::CallbackGripper, &(*kinova_bridge_ptr));
        ROS_INFO("Subscribed gripper_target");

        auto loop_start_time = std::chrono::high_resolution_clock::now();
        auto last_print_time = loop_start_time;
        size_t loop_count = 0;
        const double print_interval = 1.0; // Print every 1 second

        while (ros::ok())
        {

            ros::spinOnce();
            kinova_bridge_ptr->RefreshCommand();
            kinova_bridge_ptr->Publish();

            loop_count++;

            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - last_print_time;

            if (elapsed.count() >= print_interval)
            {
                double avg_period_ms = (elapsed.count() / loop_count) * 1000.0;
                double avg_frequency_hz = loop_count / elapsed.count();

                ROS_INFO("Main loop stats: Avg period: %.3f ms, Avg frequency: %.1f Hz",
                         avg_period_ms, avg_frequency_hz);

                last_print_time = now;
                loop_count = 0;
            }
        }

        return 0;

    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Controller failed: %s", e.what());
        ros::shutdown();

        return -1;
    }

}
