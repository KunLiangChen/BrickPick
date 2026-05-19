#include "brickpick/approach_action.hpp"

using namespace std::chrono_literals;

namespace BT {

ApproachAction::ApproachAction(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), request_sent_(false)
{
    node_ = rclcpp::Node::make_shared(name);

    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/approach_node/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            latest_status_ = msg->data;
        });

    start_client_ = node_->create_client<std_srvs::srv::Trigger>(
        "/approach_node/start");

    if (!start_client_->wait_for_service(3s)) {
        RCLCPP_WARN(node_->get_logger(),
                    "/approach_node/start service not ready, will retry on tick");
    }
}

BT::NodeStatus ApproachAction::onStart()
{
    getInput("timeout", timeout_sec_);
    start_time_ = std::chrono::steady_clock::now();
    latest_status_ = "";
    request_sent_ = false;

    RCLCPP_INFO(node_->get_logger(), "Calling /approach_node/start ...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = start_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future, 2s)
        == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(),
                        "Python node started, entering RUNNING");
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(node_->get_logger(),
                         "Service rejected: %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "/approach_node/start call timed out or no response");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ApproachAction::onRunning()
{
    if (!request_sent_)
        return BT::NodeStatus::FAILURE;

    rclcpp::spin_some(node_);

    if (latest_status_ == "SUCCESS") {
        RCLCPP_INFO(node_->get_logger(),
                    "Received Approach SUCCESS signal, task complete");
        return BT::NodeStatus::SUCCESS;
    }

    if (latest_status_.find("FAILURE") != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "Approach node reported FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed > std::chrono::duration<double>(timeout_sec_)) {
        RCLCPP_WARN(node_->get_logger(),
                    "Approach node timed out, reporting FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void ApproachAction::onHalted()
{
    request_sent_ = false;
    RCLCPP_WARN(node_->get_logger(), "Approach action halted by tree");
}

} // namespace BT
