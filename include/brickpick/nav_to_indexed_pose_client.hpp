#pragma once

#include <string>
#include <vector>
#include <utility>
#include <future>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavToIndexedPoseClient : public BT::AsyncActionNode
{
public:
    NavToIndexedPoseClient(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("current_index"),
            BT::InputPort<std::vector<std::pair<double, double>>>("regions"),
            BT::InputPort<std::string>("frame_id", "map")
        };
    }

private:
    // AsyncActionNode 必须实现的三个方法：
    // 1. 节点第一次被 tick 时调用
    BT::NodeStatus on_start() override;

    // 2. 节点返回 RUNNING 期间，会在后台线程不断循环调用
    BT::NodeStatus on_running() override;

    // 3. 如果树被外部强行中止（比如父节点失败），调用此方法取消导航
    void on_halted() override;

    // ROS2 相关成员变量
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

    // 用于接收 Nav2 最终结果的机制 (Promise/Future)
    std::shared_ptr<std::promise<rclcpp_action::ResultCode>> result_promise_;
    std::shared_future<rclcpp_action::ResultCode> result_future_;

    // 保存 GoalHandle，以便在 on_halted() 中取消目标
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
};