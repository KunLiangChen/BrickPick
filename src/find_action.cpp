#include "brickpick/find_action.hpp"

using namespace std::chrono_literals;

namespace BT {

FindAction::FindAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), request_sent_(false) {
    // 为当前 Action 创建独立的 ROS2 节点，避免与 BT 执行器共享执行流
    node_ = rclcpp::Node::make_shared("find_action_ros_node");
    
    // 订阅 Python 节点的状态话题
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/find_node/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            latest_status_ = msg->data;
        });

    // 创建 Service 客户端
    start_client_ = node_->create_client<std_srvs::srv::Trigger>("/find_node/start");

    if (!start_client_->wait_for_service(3s)) {
        node_->get_logger().warn("⚠️ /find_node/start 服务暂未就绪，将在首次 tick 时重试");
    }
}

BT::NodeStatus FindAction::onStart() {
    getInput("timeout", timeout_sec_);
    start_time_ = std::chrono::steady_clock::now();
    latest_status_ = "";
    request_sent_ = false;

    node_->get_logger().info(" 调用 /find_node/start 触发搜索...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = start_client_->async_send_request(request);

    // 阻塞等待服务响应（最多 2s），确认 Python 节点已接收指令
    if (rclcpp::spin_until_future_complete(node_, result_future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        if (response->success) {
            node_->get_logger().info("Python 节点已启动，进入 RUNNING 状态");
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        } else {
            node_->get_logger().error("服务拒绝请求: " + response->message);
            return BT::NodeStatus::FAILURE;
        }
    } else {
        node_->get_logger().error("调用 /find_node/start 超时或无响应");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FindAction::onRunning() {
    if (!request_sent_) return BT::NodeStatus::FAILURE;

    // 🔑 关键：处理 ROS 回调，更新 latest_status_
    rclcpp::spin_some(node_);

    // 检查 Python 节点是否上报 SUCCESS
    if (latest_status_ == "SUCCESS") {
        node_->get_logger().info("🎯 收到 Find SUCCESS 信号，任务完成");
        return BT::NodeStatus::SUCCESS;
    }

    // 超时保护
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed > std::chrono::seconds(static_cast<long>(timeout_sec_))) {
        node_->get_logger().warn("⏱️ Find 节点执行超时，判定为 FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    // 保持 RUNNING，BT 会在下一次 tick 再次调用本函数
    return BT::NodeStatus::RUNNING;
}

void FindAction::onHalted() {
    request_sent_ = false;
    node_->get_logger().warn("🛑 Find 动作被上层树中止 (Halted)");
    // 如需优雅停止，可在此调用额外的 /find_node/cancel 服务
}

} // namespace BT