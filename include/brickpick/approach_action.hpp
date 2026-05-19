#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <atomic>

namespace BT {

class ApproachAction : public BT::StatefulActionNode {
public:
    ApproachAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("timeout", 20.0, "Max execution time (seconds)") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    
    std::string latest_status_;
    std::chrono::steady_clock::time_point start_time_;
    double timeout_sec_;
    bool request_sent_;

    static std::atomic<bool> instance_counter_;
};

} // namespace BT