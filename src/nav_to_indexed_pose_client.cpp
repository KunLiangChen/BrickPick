#include "brickpick/nav_to_indexed_pose_client.hpp"

NavToIndexedPoseClient::NavToIndexedPoseClient(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    // 创建一个独立于 Nav2 的 ROS2 节点，专门用于发送 Action
    node_ = std::make_shared<rclcpp::Node>("nav_to_indexed_pose_client");
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
}

BT::NodeStatus NavToIndexedPoseClient::onStart()
{
    // 1. 读取黑板数据
    BT::Expected<int> index_opt = getInput<int>("current_index");
    BT::Expected<std::vector<std::pair<double, double>>> regions_opt = getInput<std::vector<std::pair<double, double>>>("regions");
    BT::Expected<std::string> frame_id_opt = getInput<std::string>("frame_id");

    if (!index_opt || !regions_opt || !frame_id_opt) return BT::NodeStatus::FAILURE;

    int current_index = index_opt.value();
    auto regions = regions_opt.value();
    
    if (current_index < 0 || static_cast<size_t>(current_index) >= regions.size()) return BT::NodeStatus::FAILURE;

    // 2. 检查 Nav2 服务器是否在线
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available!");
        return BT::NodeStatus::FAILURE;
    }

    // 3. 组装目标坐标
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = frame_id_opt.value();
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.pose.position.x = regions[current_index].first;
    goal_msg.pose.pose.position.y = regions[current_index].second;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(node_->get_logger(), "Sending goal to Nav2: index %d (%.2f, %.2f)", 
                current_index, goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    // 4. 设置接收结果的 Promise/Future
    result_promise_ = std::make_shared<std::promise<rclcpp_action::ResultCode>>();
    result_future_ = result_promise_->get_future();

    // 5. 定义 Nav2 返回结果时的回调函数
    auto result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
        // 当 Nav2 完成（无论成功失败），将结果码放入 promise，唤醒 on_running
        this->result_promise_->set_value(result.code);
    };

    // 6. 发送目标
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = result_callback;

    action_client_->async_send_goal(goal_msg, send_goal_options);

    // 告诉行为树："目标已发送，我正在执行中"
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavToIndexedPoseClient::onRunning()
{
    // 这个函数会在一个后台线程中不断被调用
    // 我们检查 future 的状态，如果 Nav2 还没回复，这里会非阻塞地立刻返回 RUNNING
    if (result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        // Nav2 回复了！获取结果
        rclcpp_action::ResultCode code = result_future_.get();
        
        if (code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Nav2 succeeded!");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Nav2 failed or was canceled!");
            return BT::NodeStatus::FAILURE;
        }
    }

    // 还没回复，继续挂起树
    return BT::NodeStatus::RUNNING;
}

void NavToIndexedPoseClient::onHalted()
{
    // 如果树因为某种原因被中断了（比如用户按了停止，或者更上层的节点失败了），
    // 必须手动取消 Nav2 的目标，否则机器人会一直往前跑！
    if (goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "Canceling Nav2 goal...");
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
}