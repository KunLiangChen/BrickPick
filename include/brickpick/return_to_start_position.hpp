#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <thread>
#include <future>

class ReturnToStartPosition : public BT::StatefulActionNode
{
public:
    ReturnToStartPosition(const std::string &name, const BT::NodeConfiguration &config);
    ~ReturnToStartPosition();

    // 🌟 返回起点不需要任何输入参数，所以这里返回空列表
    static BT::PortsList providedPorts() {
        return {};
    }

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_thread_;
    std::atomic<bool> is_spinning_;

    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_ptr<std::promise<rclcpp_action::ResultCode>> result_promise_;
    std::future<rclcpp_action::ResultCode> result_future_;
};