#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <thread>
#include <future>
#include <atomic>

class NavToIndexedPoseClient : public BT::StatefulActionNode
{
public:
    NavToIndexedPoseClient(const std::string &name, const BT::NodeConfiguration &config);
    ~NavToIndexedPoseClient() override;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("current_index"),
            BT::InputPort<std::vector<std::pair<double, double>>>("regions"),
            BT::InputPort<std::string>("frame_id")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // ROS 2 节点与 Action Client
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    
    // 保存 Goal Handle 用于取消任务
    using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    std::shared_ptr<GoalHandle> goal_handle_;

    // 异步结果接收
    std::shared_ptr<std::promise<rclcpp_action::ResultCode>> result_promise_;
    std::shared_future<rclcpp_action::ResultCode> result_future_;

    // 生命周期管理：独立执行器与后台线程
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_thread_;
    std::atomic<bool> is_spinning_;
};