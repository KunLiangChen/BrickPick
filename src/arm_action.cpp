#include "brickpick/arm_action.hpp"

using namespace std::chrono_literals;

namespace BT {

ArmAction::ArmAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), request_sent_(false) {
    // 为 Action 创建独立的 ROS2 节点
    node_ = rclcpp::Node::make_shared("arm_action_ros_node");
    
    // 订阅 Python 节点的状态话题
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm_preset_node/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            latest_status_ = msg->data;
        });

    // 创建 Service 客户端
    start_client_ = node_->create_client<std_srvs::srv::Trigger>("/arm_preset_node/start");

    if (!start_client_->wait_for_service(3s)) {
        RCLCPP_WARN(node_->get_logger(),"/arm_preset_node/start 服务暂未就绪，将在 tick 时重试");
    }
}

BT::NodeStatus ArmAction::onStart() {
    getInput("timeout", timeout_sec_);
    start_time_ = std::chrono::steady_clock::now();
    latest_status_ = "";
    request_sent_ = false;

    RCLCPP_INFO(node_->get_logger(),"调用 /arm_preset_node/start 触发机械臂序列...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = start_client_->async_send_request(request);

    // 阻塞等待服务响应（最多 2s），确认 Python 节点已接收指令并启动后台线程
    if (rclcpp::spin_until_future_complete(node_, result_future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(),"机械臂序列已触发，进入 RUNNING 状态");
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "服务拒绝请求: %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "调用 /arm_preset_node/start 超时或无响应");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ArmAction::onRunning() {
    if (!request_sent_) return BT::NodeStatus::FAILURE;

    // 🔑 关键：非阻塞处理 ROS 回调，实时更新 latest_status_
    rclcpp::spin_some(node_);

    // 检查 Python 节点是否上报 SUCCESS
    if (latest_status_ == "SUCCESS") {
        RCLCPP_INFO(node_->get_logger(),"机械臂序列执行成功");
        return BT::NodeStatus::SUCCESS;
    }
    
    // 兼容 FAILURE 状态上报
    if (latest_status_.find("FAILURE") != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "机械臂序列执行失败");
        return BT::NodeStatus::FAILURE;
    }

    // 超时保护（机械臂动作通常较慢，默认 15s 可被 XML 覆盖）
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed > std::chrono::duration<double>(timeout_sec_)) {
        RCLCPP_WARN(node_->get_logger(),"机械臂序列执行超时，判定为 FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    // 保持 RUNNING，等待下一次 tick
    return BT::NodeStatus::RUNNING;
}

void ArmAction::onHalted() {
    request_sent_ = false;
    RCLCPP_WARN(node_->get_logger(),"Arm 动作被上层树中止 (Halted)");
    // 若后续增加 /arm_preset_node/cancel 服务，可在此调用实现安全急停
}

} // namespace BT