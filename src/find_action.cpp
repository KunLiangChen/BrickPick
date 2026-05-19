#include "brickpick/find_action.hpp"

using namespace std::chrono_literals;

namespace BT {

FindAction::FindAction(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), request_sent_(false)
{
    node_ = rclcpp::Node::make_shared(name);

    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/find_node/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            latest_status_ = msg->data;
        });

    start_client_ = node_->create_client<std_srvs::srv::Trigger>(
        "/find_node/start");

    if (!start_client_->wait_for_service(3s)) {
        RCLCPP_WARN(node_->get_logger(),
                    "/find_node/start service not ready, will retry on tick");
    }
}

BT::NodeStatus FindAction::onStart()
{
    getInput("timeout", timeout_sec_);
    start_time_ = std::chrono::steady_clock::now();
    latest_status_ = "";
    request_sent_ = false;

    RCLCPP_INFO(node_->get_logger(), "Calling /find_node/start ...");
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
                     "/find_node/start call timed out or no response");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FindAction::onRunning()
{
    if (!request_sent_)
        return BT::NodeStatus::FAILURE;

    rclcpp::spin_some(node_);

    if (latest_status_ == "SUCCESS") {
        RCLCPP_INFO(node_->get_logger(),
                    "Received Find SUCCESS signal, task complete");
        return BT::NodeStatus::SUCCESS;
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed > std::chrono::seconds(static_cast<long>(timeout_sec_))) {
        RCLCPP_WARN(node_->get_logger(),
                    "Find node timed out, reporting FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void FindAction::onHalted()
{
    request_sent_ = false;
    RCLCPP_WARN(node_->get_logger(), "Find action halted by tree");
}

} // namespace BT
