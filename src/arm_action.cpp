#include "brickpick/arm_action.hpp"

using namespace std::chrono_literals;

namespace BT {

ArmAction::ArmAction(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), request_sent_(false)
{
    node_ = rclcpp::Node::make_shared(name);

    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm_preset_node/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            latest_status_ = msg->data;
        });

    start_client_ = node_->create_client<std_srvs::srv::Trigger>(
        "/arm_preset_node/start");

    if (!start_client_->wait_for_service(3s)) {
        RCLCPP_WARN(node_->get_logger(),
                    "/arm_preset_node/start service not ready, will retry on tick");
    }
}

BT::NodeStatus ArmAction::onStart()
{
    getInput("timeout", timeout_sec_);
    start_time_ = std::chrono::steady_clock::now();
    latest_status_ = "";
    request_sent_ = false;

    RCLCPP_INFO(node_->get_logger(), "Calling /arm_preset_node/start ...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = start_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future, 2s)
        == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(),
                        "Arm sequence triggered, entering RUNNING");
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(node_->get_logger(),
                         "Service rejected: %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "/arm_preset_node/start call timed out or no response");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ArmAction::onRunning()
{
    if (!request_sent_)
        return BT::NodeStatus::FAILURE;

    rclcpp::spin_some(node_);

    if (latest_status_ == "SUCCESS") {
        RCLCPP_INFO(node_->get_logger(), "Arm sequence completed successfully");
        return BT::NodeStatus::SUCCESS;
    }

    if (latest_status_.find("FAILURE") != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "Arm sequence reported FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed > std::chrono::duration<double>(timeout_sec_)) {
        RCLCPP_WARN(node_->get_logger(),
                    "Arm sequence timed out, reporting FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void ArmAction::onHalted()
{
    request_sent_ = false;
    RCLCPP_WARN(node_->get_logger(), "Arm action halted by tree");
}

} // namespace BT
